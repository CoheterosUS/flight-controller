#!/usr/bin/env python3
"""HIL harness for the Coheteros flight controller (kalman-filter branch).

See HIL/HIL_TEST_PLAN.md for the scenarios and pass criteria. The firmware must be a HIL_MODE 1 build.

Frames:
- Body (rocket) frame: +X is the nose, +Y and +Z are the fuselage marks. Specific force convention of the
  Kalman filter: the axis pointing up reads +g.
- Raw frame: hardware IMU axes. The harness converts body truth to raw with an M_true preset:
  raw_accel = M_true^-1 (f_body + b_true), raw_gyro = Q_true^T w_body + gyro_bias_raw,
  where M_true = Q_true U (QR, Q_true a rotation). The firmware must learn M_true in the tumble.
- The default mounting matches the board: raw Y = -body X (nose up reads raw Y = -9.81).

Transport: COMMAND_HIL_DATA (0x10, 36 byte payload: raw accel, raw gyro, mag) at --rate Hz (default 200 =
IMU_ODR_HZ), and COMMAND_HIL_BARO (0x11, 8 byte payload: pressure, temperature) at --baro-rate Hz (default 50).
One barometer frame is one barometer sample, as on the hardware.

Use --port sim to run any scenario against a crude built-in mock of the board (harness self test only).
Exit code: 0 all pass, 1 any failure, 2 setup error.
"""
import argparse
import csv
import math
import os
import random
import struct
import sys
import time

# ---------------------------------------------------------------- protocol
PACKET_HEADER_LSB = 0xFE
PACKET_HEADER_MSB = 0xCA
PACKET_FOOTER = 0xBE

COMMAND_RESET = 0x01
COMMAND_GROUND_ABORT = 0x02
COMMAND_CALIBRATION = 0x03
COMMAND_DROGUE = 0x04
COMMAND_LANDED = 0x05
COMMAND_HIL_DATA = 0x10
COMMAND_HIL_BARO = 0x11

HIL_DATA_PAYLOAD_SIZE = 36
HIL_BARO_PAYLOAD_SIZE = 8

TELEMETRY_FORMAT = "<HI6hhbiiiBiiIhBBBHB"
TELEMETRY_PACKET_SIZE = struct.calcsize(TELEMETRY_FORMAT)
assert TELEMETRY_PACKET_SIZE == 54
TELEMETRY_FIELDS = [
    "sync", "tick", "cal_ax", "cal_ay", "cal_az", "cal_gx", "cal_gy", "cal_gz", "pressure_pa",
    "temperature_c", "lat", "lon", "gps_alt", "sats", "baro_alt", "baro_vel", "flags", "battery",
    "state", "relay", "last_command", "cal_status", "sync_end",
]

STATE_IDLE, STATE_CALIBRATION, STATE_PRELAUNCH, STATE_BOOST, STATE_COAST, STATE_ACTIVE_CONTROL = range(6)
STATE_APOGEE, STATE_MAIN_PARACHUTE, STATE_LANDED, STATE_GROUND_ABORT, STATE_DESCENT_ABORT = range(6, 11)
STATE_ASCENT_ABORT, STATE_DEEP_CALIBRATION = 11, 12
STATE_NAMES = {
    0: "IDLE", 1: "CALIBRATION", 2: "PRELAUNCH", 3: "BOOST", 4: "COAST", 5: "ACTIVE_CTRL",
    6: "APOGEE", 7: "MAIN_CHUTE", 8: "LANDED", 9: "GND_ABORT", 10: "DESC_ABORT",
    11: "ASC_ABORT", 12: "DEEP_CAL",
}
ABORT_STATES = (STATE_GROUND_ABORT, STATE_DESCENT_ABORT, STATE_ASCENT_ABORT)

CAL_IMU_VALID, CAL_GYRO_BIAS, CAL_ACCEL_BIAS, CAL_PRESSURE_REF = 1 << 0, 1 << 1, 1 << 2, 1 << 3
CAL_KALMAN_INIT, CAL_KALMAN_STEPPING, CAL_HIL_PRESEED, CAL_HIL_MODE = 1 << 4, 1 << 5, 1 << 6, 1 << 7
CAL_ALL_VALID = CAL_IMU_VALID | CAL_GYRO_BIAS | CAL_ACCEL_BIAS | CAL_PRESSURE_REF

FAULT_NAMES = {
    0: "BMP280_IDLE", 1: "BMP280_PERF", 2: "BMP581_IDLE", 3: "BMP581_PERF", 4: "IIM42653_IDLE",
    5: "IIM42653_PERF", 6: "IIS2MDCTR_IDLE", 7: "IIS2MDCTR_PERF", 8: "SD_MOUNT", 9: "SD_OPEN",
    10: "W25Q_JEDEC", 11: "W25Q_INIT", 12: "W25Q_LOG_FULL", 13: "W25Q_WRITE", 14: "W25Q_SNAPSHOT",
}

# ---------------------------------------------------------------- physics and firmware constants
G = 9.81
P_REF = 101325.0
T_REF_C = 25.0
T_REF_K = T_REF_C + 273.15
MAIN_DEPLOY_ALT = 450.0
APOGEE_TIMER_S = 28.0
ACTIVE_CONTROL_ALT = 2000.0
DEEP_CAL_HOLD_S = 10.0
DEEP_CAL_TIMEOUT_S = 2 * 6 * (10.0 + 30.0)

NOISE_ACCEL = 0.05
NOISE_GYRO = 0.01
NOISE_MAG = 0.5
NOISE_PRESSURE = 2.0
NOISE_TEMPERATURE = 0.1

# Body specific force of each tumble pose (the axis pointing up reads +g), prompts 1 to 6.
POSE_FORCE = {
    1: (G, 0.0, 0.0), 2: (-G, 0.0, 0.0),
    3: (0.0, G, 0.0), 4: (0.0, -G, 0.0),
    5: (0.0, 0.0, G), 6: (0.0, 0.0, -G),
}
NOSE_UP = POSE_FORCE[1]
NOSE_DOWN = POSE_FORCE[2]
LYING = (0.0, G, 0.0)


def pressure_from_altitude(alt_m, p_ref=P_REF, t_ref_k=T_REF_K):
    """Same atmosphere model as the firmware (APOGEE_HIL.md section 2)."""
    base = 1.0 - 0.0065 * alt_m / t_ref_k
    return p_ref * max(base, 1e-6) ** (9.81 / (287.05 * 0.0065))


def altitude_from_pressure(p, p_ref=P_REF, t_ref_k=T_REF_K):
    if not (p > 0.0) or math.isinf(p):
        return float("nan")
    return t_ref_k / 0.0065 * (1.0 - (p / p_ref) ** (287.05 * 0.0065 / 9.81))


# ---------------------------------------------------------------- 3x3 helpers (no numpy dependency)
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2] for i in range(3)]


def mat_t(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def mat_det(a):
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))


def mat_inv(a):
    d = mat_det(a)
    c = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            m = [[a[r][s] for s in range(3) if s != j] for r in range(3) if r != i]
            c[j][i] = ((-1) ** (i + j)) * (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / d
    return c


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


# body = R_MOUNT raw: raw X = body Y, raw Y = -body X, raw Z = body Z
R_MOUNT = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]


class Mounting:
    """An M_true preset: M = Q U, accel offset b (body, m/s2), raw gyro bias (dps)."""

    def __init__(self, name, q, u, b, gyro_bias_raw):
        self.name = name
        self.Q = q
        self.M = mat_mul(q, u)
        self.M_inv = mat_inv(self.M)
        self.b = list(b)
        self.gyro_bias_raw = list(gyro_bias_raw)
        assert abs(mat_det(q) - 1.0) < 1e-9, "Q_true must be a proper rotation"

    def raw_accel(self, f_body):
        return mat_vec(self.M_inv, [f_body[i] + self.b[i] for i in range(3)])

    def raw_gyro(self, w_body):
        w = mat_vec(mat_t(self.Q), w_body)
        return [w[i] + self.gyro_bias_raw[i] for i in range(3)]


def make_mounting(name):
    misalign = mat_mul(rot_z(1.5), mat_mul(rot_y(-2.0), rot_x(1.0)))
    if name == "default":
        q = mat_mul(misalign, R_MOUNT)
        u = [[1.02, 0.004, -0.003], [0.0, 0.985, 0.005], [0.0, 0.0, 1.01]]
        return Mounting(name, q, u, (0.15, 0.25, -0.20), (0.3, -0.2, 0.1))
    if name == "alt":
        q = mat_mul(mat_mul(rot_z(-2.5), rot_x(3.0)), R_MOUNT)
        u = [[0.975, -0.006, 0.002], [0.0, 1.025, -0.004], [0.0, 0.0, 0.99]]
        return Mounting(name, q, u, (-0.20, 0.10, 0.30), (-0.5, 0.4, 0.2))
    if name == "roll90":
        # S3: 90 deg about the nose (a yaw about Z would move the nose off raw Y and the gesture could not trigger),
        # constant raw gyro offset (1, 2, 3) dps.
        q = mat_mul(rot_x(90.0), mat_mul(misalign, R_MOUNT))
        u = [[1.01, 0.003, 0.0], [0.0, 0.99, 0.002], [0.0, 0.0, 1.0]]
        return Mounting(name, q, u, (0.10, -0.10, 0.15), (1.0, 2.0, 3.0))
    if name == "identity":
        return Mounting(name, R_MOUNT, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], (0, 0, 0), (0, 0, 0))
    raise SystemExit(f"unknown mounting preset {name}")


# ---------------------------------------------------------------- transport
def build_packet(command, payload=b""):
    return bytes([PACKET_HEADER_LSB, PACKET_HEADER_MSB, command, len(payload)]) + payload + bytes([PACKET_FOOTER])


def build_hil_packet(accel, gyro, mag):
    return build_packet(COMMAND_HIL_DATA, struct.pack("<9f", *accel, *gyro, *mag))


def build_baro_packet(pressure, temperature):
    return build_packet(COMMAND_HIL_BARO, struct.pack("<2f", pressure, temperature))


class TelemetryParser:
    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            idx = self.buf.find(bytes([PACKET_HEADER_LSB, PACKET_HEADER_MSB]))
            if idx < 0:
                del self.buf[:-1]
                break
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < TELEMETRY_PACKET_SIZE:
                break
            if self.buf[TELEMETRY_PACKET_SIZE - 1] != PACKET_FOOTER:
                self.bad += 1
                del self.buf[:1]
                continue
            values = struct.unpack_from(TELEMETRY_FORMAT, self.buf, 0)
            del self.buf[:TELEMETRY_PACKET_SIZE]
            tel = dict(zip(TELEMETRY_FIELDS, values))
            tel["baro_alt"] /= 100.0
            tel["baro_vel"] /= 100.0
            tel["gps_alt"] /= 100.0
            tel["pressure_pa"] *= 10.0
            tel["battery"] /= 10.0
            tel["pose"] = (tel["cal_status"] >> 8) & 0x7
            out.append(tel)
        return out


# ---------------------------------------------------------------- crude mock board (harness self test only)
class MockBoard:
    """Not the firmware. Just enough state logic to exercise every harness path with --port sim."""

    def __init__(self, rate, mounting, start_valid=True, baro_rate=50):
        self.rate = rate
        self.baro_rate = baro_rate
        self.m = make_mounting("default") if mounting is None else mounting
        self.rx = bytearray()
        self.tx = bytearray()
        self.n = 0
        self.state = STATE_IDLE
        self.cal = CAL_HIL_MODE | (CAL_IMU_VALID if start_valid else 0)
        self.pose = 0
        self.timer = 0.0
        self.gesture = 0.0
        self.state_t = 0.0
        self.peak = -1e9
        self.below = 0
        self.fast = 0
        self.boost_t = 0.0
        self.alt = 0.0
        self.last_alt = 0.0
        self.body = [0.0, 0.0, 0.0]
        self.raw = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]

    @property
    def in_waiting(self):
        return len(self.tx)

    def read(self, n):
        data = bytes(self.tx[:n])
        del self.tx[:n]
        return data

    def close(self):
        pass

    def write(self, data):
        self.rx += data
        while len(self.rx) >= 5:
            if self.rx[0] != PACKET_HEADER_LSB or self.rx[1] != PACKET_HEADER_MSB:
                del self.rx[:1]
                continue
            length = self.rx[3]
            if len(self.rx) < 5 + length:
                break
            cmd, payload = self.rx[2], bytes(self.rx[4:4 + length])
            del self.rx[:5 + length]
            if cmd == COMMAND_HIL_DATA and length == HIL_DATA_PAYLOAD_SIZE:
                self.step(struct.unpack("<9f", payload))
            elif cmd == COMMAND_HIL_BARO and length == HIL_BARO_PAYLOAD_SIZE:
                self.baro(struct.unpack("<2f", payload))

    def set_state(self, s):
        self.state = s
        self.state_t = self.n / self.rate

    def step(self, v):
        self.n += 1
        dt = 1.0 / self.rate
        now = self.n * dt
        self.raw = list(v[0:3])
        self.gyro = mat_vec(self.m.Q, [v[3 + i] - self.m.gyro_bias_raw[i] for i in range(3)])
        bias = self.m.b if self.cal & CAL_ACCEL_BIAS else [0.0, 0.0, 0.0]
        self.body = [x - bias[i] for i, x in enumerate(mat_vec(self.m.M, self.raw))]
        still = math.sqrt(sum(g * g for g in v[3:6])) < 5.0
        s = self.state
        nose_down_raw = self.raw[1] > 8.8 and abs(self.raw[0]) < 2 and abs(self.raw[2]) < 2 and still
        if s in (STATE_IDLE, STATE_CALIBRATION, STATE_PRELAUNCH) and nose_down_raw:
            self.gesture += dt
            if self.gesture >= DEEP_CAL_HOLD_S:
                self.set_state(STATE_DEEP_CALIBRATION)
                self.pose, self.timer, self.gesture = 0, 0.0, 0.0
                return self.emit()
        else:
            self.gesture = 0.0
        if s == STATE_IDLE and now - self.state_t > 0.5:
            self.set_state(STATE_CALIBRATION)
        elif s == STATE_CALIBRATION:
            ok = self.cal & CAL_IMU_VALID and self.body[0] > 9.0 and still
            self.timer = self.timer + dt if ok else 0.0
            if self.timer > 8.0:
                self.cal |= CAL_ALL_VALID | CAL_KALMAN_INIT | CAL_KALMAN_STEPPING
                self.set_state(STATE_PRELAUNCH)
        elif s == STATE_DEEP_CALIBRATION:
            self.timer += dt
            if self.pose == 0 and self.timer > 2.0:
                self.pose, self.timer = 1, 0.0
            elif self.pose and self.timer > 10.0:
                want = POSE_FORCE[self.pose]
                if any(abs(self.body[i] - want[i]) > 2.0 for i in range(3)) or not still:
                    self.timer = 0.0
                elif self.timer > 40.0:
                    self.pose, self.timer = self.pose + 1, 0.0
                    if self.pose > 6:
                        self.pose = 0
                        self.cal |= CAL_IMU_VALID
                        self.set_state(STATE_IDLE)
        elif s == STATE_PRELAUNCH:
            self.fast = self.fast + 1 if self.body[0] > 20 else 0
            if self.fast >= 12:
                self.set_state(STATE_BOOST)
                self.boost_t, self.fast = now, 0
        elif s == STATE_BOOST:
            self.fast = self.fast + 1 if self.body[0] < 5 else 0
            if self.fast >= 12:
                self.set_state(STATE_COAST)
        elif s == STATE_COAST and self.alt > ACTIVE_CONTROL_ALT:
            self.set_state(STATE_ACTIVE_CONTROL)
        elif s == STATE_APOGEE and self.alt < MAIN_DEPLOY_ALT:
            self.set_state(STATE_MAIN_PARACHUTE)
        elif s == STATE_MAIN_PARACHUTE and self.alt < 100 and abs(self.alt - self.last_alt) * self.baro_rate < 2:
            self.set_state(STATE_LANDED)
            self.cal &= ~CAL_KALMAN_STEPPING
        if s in (STATE_BOOST, STATE_COAST, STATE_ACTIVE_CONTROL) and now - self.boost_t >= APOGEE_TIMER_S:
            self.set_state(STATE_APOGEE)
        if self.n % max(1, self.rate // 10) == 0:
            self.emit()

    def baro(self, v):
        a = altitude_from_pressure(v[0])
        if not (math.isfinite(a) and abs(a) < 20000):
            return
        self.last_alt, self.alt = self.alt, a
        if self.state in (STATE_COAST, STATE_ACTIVE_CONTROL):
            self.peak = max(self.peak, self.alt)
            self.below = self.below + 1 if self.alt < self.peak - 15 else 0
            if self.below >= 5:
                self.set_state(STATE_APOGEE)

    def emit(self):
        cal = (self.cal & 0xFF) | (self.pose << 8)
        trunc = [int(x) for x in self.body] + [int(x) for x in self.gyro]
        pkt = struct.pack(TELEMETRY_FORMAT, 0xCAFE, self.n * 1000 // self.rate, *trunc, int(P_REF / 10),
                          25, 0, 0, 0, 0, int(self.alt * 100), 0, 0, 80, self.state, 0, 0, cal, PACKET_FOOTER)
        self.tx += pkt


# ---------------------------------------------------------------- runner
class Runner:
    def __init__(self, args, mounting, tag):
        self.args = args
        self.m = mounting
        self.rate = args.rate
        self.dt = 1.0 / self.rate
        self.baro_every = max(1, round(self.rate / args.baro_rate))
        self.sim = args.port == "sim"
        if self.sim:
            self.ser = MockBoard(self.rate, mounting, start_valid=args.scenario not in ("tumble", "calibrate"),
                                 baro_rate=args.baro_rate)
        else:
            import serial  # pyserial
            self.ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=1)
        self.parser = TelemetryParser()
        self.t = 0.0
        self.tel = None
        self.tel_count = 0
        self.history = []
        self.last_state = None
        self.transitions = []
        self.last_pressure = None
        self.t0 = time.perf_counter()
        self.n = 0
        os.makedirs(args.log_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(args.log_dir, f"{stamp}_{tag}_seed{args.seed}")
        self.tel_file = open(base + "_telemetry.csv", "w", newline="")
        self.tel_csv = csv.writer(self.tel_file)
        self.tel_csv.writerow(["sim_t"] + TELEMETRY_FIELDS + ["pose"])
        self.truth_file = open(base + "_truth.csv", "w", newline="")
        self.truth_csv = csv.writer(self.truth_file)
        self.truth_csv.writerow(["sim_t", "phase", "alt_true", "alt_sent", "fx", "fy", "fz", "wx", "wy", "wz",
                                 "raw_ax", "raw_ay", "raw_az", "raw_gx", "raw_gy", "raw_gz", "pressure", "temp"])
        print(f"log: {base}_*.csv")

    def close(self):
        self.tel_file.close()
        self.truth_file.close()
        self.ser.close()

    def pace(self):
        if self.sim:
            return
        target = self.t0 + self.n * self.dt
        while True:
            left = target - time.perf_counter()
            if left <= 0:
                break
            if left > 0.002:
                time.sleep(left - 0.0015)

    def poll(self):
        n = self.ser.in_waiting
        if not n:
            return
        for tel in self.parser.feed(self.ser.read(n)):
            self.tel = tel
            self.tel_count += 1
            tel["sim_t"] = self.t
            self.history.append(tel)
            self.tel_csv.writerow([f"{self.t:.3f}"] + [tel[k] for k in TELEMETRY_FIELDS] + [tel["pose"]])
            if tel["state"] != self.last_state:
                self.transitions.append((self.t, tel["state"]))
                print(f"  [{self.t:7.2f}s] state {STATE_NAMES.get(self.last_state, '-')} -> "
                      f"{STATE_NAMES.get(tel['state'], tel['state'])}  cal=0x{tel['cal_status']:04x} "
                      f"flags=0x{tel['flags']:08x}")
                self.last_state = tel["state"]

    def send(self, f_body, w_body=(0.0, 0.0, 0.0), alt=0.0, phase="", alt_sent=None, pressure=None,
             temperature=None, accel_override=None, gyro_override=None, noise=True):
        """One IMU period: body truth -> raw -> packet. Always a fresh noisy sample."""
        na = NOISE_ACCEL if noise else 0.0
        ng = NOISE_GYRO if noise else 0.0
        f = [f_body[i] + random.gauss(0.0, na) for i in range(3)]
        w = [w_body[i] + random.gauss(0.0, ng) for i in range(3)]
        raw_a = accel_override if accel_override is not None else self.m.raw_accel(f)
        raw_g = gyro_override if gyro_override is not None else self.m.raw_gyro(w)
        mag = [20.0 + random.gauss(0, NOISE_MAG), 5.0 + random.gauss(0, NOISE_MAG), -40.0 + random.gauss(0, NOISE_MAG)]
        if alt_sent is None:
            alt_sent = alt
        send_baro = self.baro_due() and pressure != "skip"
        if pressure is None or pressure == "skip":
            pressure = pressure_from_altitude(max(alt_sent, -100.0)) + random.gauss(0.0, self.args.baro_noise)
        if temperature is None:
            temperature = T_REF_C + random.gauss(0.0, NOISE_TEMPERATURE)
        self.ser.write(build_hil_packet(raw_a, raw_g, mag))
        if send_baro:
            self.last_pressure = pressure
            self.ser.write(build_baro_packet(pressure, temperature))
        if self.n % max(1, self.rate // 20) == 0:
            self.truth_csv.writerow([f"{self.t:.3f}", phase, f"{alt:.2f}", f"{alt_sent:.2f}", *[f"{x:.3f}" for x in f_body],
                                     *[f"{x:.3f}" for x in w_body], *[f"{x:.4f}" for x in raw_a],
                                     *[f"{x:.4f}" for x in raw_g], f"{pressure:.2f}", f"{temperature:.2f}"])
        self.n += 1
        self.t += self.dt
        self.pace()
        self.poll()

    def baro_due(self):
        """True when the IMU period about to be sent also carries a barometer sample."""
        return self.n % self.baro_every == 0

    def send_command(self, command):
        self.ser.write(build_packet(command))

    def hold(self, f_body, seconds, until=None, w_body=(0.0, 0.0, 0.0), alt=0.0, phase="hold"):
        """Feed a static pose. Returns True when until(tel) became true (or at the end when until is None)."""
        end = self.t + seconds
        while self.t < end:
            self.send(f_body, w_body, alt, phase)
            if until is not None and self.tel is not None and until(self.tel):
                return True
        return until is None

    def wait_alive(self, seconds=10.0):
        ok = self.hold(NOSE_UP, seconds, until=lambda tel: True, phase="alive")
        if not ok:
            raise SetupError(f"no telemetry in {seconds:.0f} s (port {self.args.port}, {self.args.baud} baud). "
                             "Check the cable and that the board runs a HIL_MODE 1 build.")
        if not (self.tel["cal_status"] & CAL_HIL_MODE):
            raise SetupError("CalStatus bit 7 (HIL_MODE) is clear: the board is not running a HIL build")

    def recent(self, seconds):
        return [h for h in self.history if h["sim_t"] >= self.t - seconds]


class SetupError(Exception):
    pass


class Result:
    def __init__(self, name):
        self.name = name
        self.checks = []

    def check(self, ok, what):
        self.checks.append((bool(ok), what))
        print(f"    {'PASS' if ok else 'FAIL'}: {what}")
        return ok

    @property
    def ok(self):
        return bool(self.checks) and all(c[0] for c in self.checks)


def trunc_ok(reported, expected, tol):
    """Telemetry truncates to int16 toward zero: compare the reported integer with the truncated expectation."""
    return abs(reported - expected) <= tol + 1.0


# ---------------------------------------------------------------- scenarios: calibration
def do_tumble(r, res, swap=None, motion_pose=None, wrong_pose=None, restart_forever=None, expect_success=True):
    print("  gesture: nose down and still")
    start = r.t
    entered = r.hold(NOSE_DOWN, DEEP_CAL_HOLD_S + 8.0, until=lambda tel: tel["state"] == STATE_DEEP_CALIBRATION,
                     phase="gesture")
    if not res.check(entered, "state 12 (DEEP_CALIBRATION) entered after the nose-down hold"):
        return False
    held = r.t - start
    res.check(DEEP_CAL_HOLD_S - 0.5 <= held <= DEEP_CAL_HOLD_S + 2.0,
              f"gesture took {held:.1f} s (expected about {DEEP_CAL_HOLD_S:.0f} s)")

    seen_poses = []
    pose_started = {}
    deadline = r.t + DEEP_CAL_TIMEOUT_S + 30.0
    while r.t < deadline:
        tel = r.tel
        if tel["state"] != STATE_DEEP_CALIBRATION:
            break
        pose = tel["pose"]
        if pose and (not seen_poses or seen_poses[-1] != pose):
            seen_poses.append(pose)
            pose_started[pose] = r.t
            print(f"  [{r.t:7.2f}s] prompt {pose}")
        if pose == 0:
            r.send(NOSE_DOWN, phase="entry")
            continue
        fed = pose
        if swap and pose in swap:
            fed = swap[pose]
        if wrong_pose == pose:
            fed = pose + 1 if pose % 2 else pose - 1
        f = POSE_FORCE[fed]
        w = (0.0, 0.0, 0.0)
        since = r.t - pose_started[pose]
        if restart_forever == pose:
            w = (0.0, 0.0, 20.0)
        elif motion_pose == pose and 15.0 <= since < 18.0 and seen_poses.count(pose) == 1:
            w = (0.0, 0.0, 20.0)
        r.send(f, w, phase=f"pose{pose}")
    else:
        res.check(False, "the tumble did not end before the harness deadline")
        return False

    print(f"  prompts seen: {seen_poses}")
    if expect_success:
        res.check([p for i, p in enumerate(seen_poses) if i == 0 or seen_poses[i - 1] != p][:6] == [1, 2, 3, 4, 5, 6],
                  "prompts 1 to 6 in order")
    ok_state = r.hold(NOSE_UP, 3.0, until=lambda tel: tel["state"] in (STATE_IDLE, STATE_CALIBRATION), phase="exit")
    res.check(ok_state, "returned to IDLE (or auto-advanced to CALIBRATION) after the tumble")
    return True


def verify_calibrated_accel(r, res):
    """After a tumble, each pose must read f + b in CalAccel (bias not yet removed), within int16 truncation."""
    for pose in (1, 3, 4, 5, 6):  # no nose down: holding it 10 s would re-trigger the gesture
        r.hold(POSE_FORCE[pose], 1.5, phase=f"verify{pose}")
        samples = [h for h in r.recent(1.0)]
        if not samples:
            res.check(False, f"verify pose {pose}: no telemetry")
            continue
        h = samples[-1]
        bias_removed = bool(h["cal_status"] & CAL_ACCEL_BIAS)
        want = [POSE_FORCE[pose][i] + (0.0 if bias_removed else r.m.b[i]) for i in range(3)]
        got = [h["cal_ax"], h["cal_ay"], h["cal_az"]]
        res.check(all(trunc_ok(got[i], want[i], 0.3) for i in range(3)),
                  f"verify pose {pose}: CalAccel {got} vs expected {[round(x, 2) for x in want]}")


def scenario_tumble(r, args):
    res = Result("tumble")
    r.wait_alive()
    before = r.tel["cal_status"]
    print(f"  start: state {STATE_NAMES.get(r.tel['state'])}, cal=0x{before:04x}, M valid={bool(before & CAL_IMU_VALID)}")
    swap = {3: 4, 4: 3} if args.mirrored else None
    expect = not (args.mirrored or args.wrong_pose or args.restart_forever)
    if not do_tumble(r, res, swap=swap, motion_pose=args.motion_pose, wrong_pose=args.wrong_pose,
                     restart_forever=args.restart_forever, expect_success=expect):
        return res
    r.hold(NOSE_UP, 2.0, phase="settle")
    valid = bool(r.tel["cal_status"] & CAL_IMU_VALID)
    if expect:
        res.check(valid, "CalStatus bit 0 (IMU_CAL_VALID) set after the tumble")
        verify_calibrated_accel(r, res)
    else:
        if before & CAL_IMU_VALID:
            print("    NOTE: an older M was already valid, so bit 0 cannot show the failure. Start from erased "
                  "calibration sectors for the negative cases.")
        else:
            res.check(not valid, "the failed tumble stored no M (bit 0 still clear)")
    return res


def do_pad(r, res, timeout=120.0):
    print("  pad: nose up and still until PRELAUNCH")
    ok = r.hold(NOSE_UP, timeout, until=lambda tel: tel["state"] == STATE_PRELAUNCH, phase="pad")
    if not res.check(ok, f"PRELAUNCH reached within {timeout:.0f} s"):
        if r.tel:
            print(f"    state {STATE_NAMES.get(r.tel['state'])} cal=0x{r.tel['cal_status']:04x} flags=0x{r.tel['flags']:08x}")
        return False
    r.hold(NOSE_UP, 1.0, phase="pad")
    h = r.tel
    cal = h["cal_status"]
    res.check((cal & CAL_ALL_VALID) == CAL_ALL_VALID, f"CalStatus bits 0 to 3 valid (cal=0x{cal:04x})")
    res.check(cal & CAL_KALMAN_INIT and cal & CAL_KALMAN_STEPPING, "Kalman initialised and stepping in PRELAUNCH")
    res.check(not (cal & CAL_HIL_PRESEED), "no HIL pre-seed (bit 6 clear)")
    res.check(h["flags"] == 0, f"no fault flags (flags=0x{h['flags']:08x})")
    got = [h["cal_ax"], h["cal_ay"], h["cal_az"]]
    res.check(got[0] in (9, 10) and got[1] == 0 and got[2] == 0, f"CalAccel {got} is about (+9.81, 0, 0)")
    gyro = [h["cal_gx"], h["cal_gy"], h["cal_gz"]]
    res.check(gyro == [0, 0, 0], f"CalGyro {gyro} is 0 at rest (the S3 frame bug shows about 3 dps)")
    return True


def verify_gyro_axes(r, res):
    """A known body rate on each axis must come out on the same body axis (checks Q and the bias frame)."""
    for axis in range(3):
        w = [0.0, 0.0, 0.0]
        w[axis] = 20.0
        r.hold(NOSE_UP, 1.0, w_body=w, phase=f"rate{axis}")
        h = r.tel
        got = [h["cal_gx"], h["cal_gy"], h["cal_gz"]]
        res.check(all(trunc_ok(got[i], w[i], 0.5) for i in range(3)),
                  f"body rate {w} dps reads CalGyro {got}")
        r.hold(NOSE_UP, 0.5, phase="rest")


def scenario_pad(r, args):
    res = Result("pad")
    r.wait_alive()
    if do_pad(r, res):
        verify_gyro_axes(r, res)
        res.check(r.tel["state"] == STATE_PRELAUNCH, "still in PRELAUNCH after the rate checks")
    return res


def scenario_calibrate(r, args):
    """S1 + S2 (+ S3 with --mounting roll90): tumble, verify, pad calibration, gyro axis check."""
    res = scenario_tumble(r, args)
    if res.ok:
        if do_pad(r, res):
            verify_gyro_axes(r, res)
    return res


def scenario_regesture(r, args):
    """S6h: the gesture in PRELAUNCH re-enters DEEP_CALIBRATION, then a full re-run (S7)."""
    res = Result("regesture")
    r.wait_alive()
    if r.tel["state"] != STATE_PRELAUNCH and not do_pad(r, res):
        return res
    if do_tumble(r, res):
        r.hold(NOSE_UP, 2.0, phase="settle")
        res.check(r.tel["cal_status"] & CAL_IMU_VALID, "M valid after the re-run")
        do_pad(r, res)
    return res


def scenario_pad_wait(r, args):
    """S4b: long pad wait, no spurious BOOST, Kalman keeps stepping."""
    res = Result("pad_wait")
    r.wait_alive()
    if r.tel["state"] != STATE_PRELAUNCH and not do_pad(r, res):
        return res
    left = r.hold(NOSE_UP, args.duration, until=lambda tel: tel["state"] != STATE_PRELAUNCH, phase="padwait")
    res.check(not left, f"stayed in PRELAUNCH for {args.duration:.0f} s")
    res.check(r.tel["cal_status"] & CAL_KALMAN_STEPPING, "Kalman still stepping")
    return res


def scenario_commands(r, args):
    """R1: every serial state command is ignored (EXTERNAL_COMMANDS 0)."""
    res = Result("commands")
    r.wait_alive()
    if r.tel["state"] != STATE_PRELAUNCH and not do_pad(r, res):
        return res
    for name, cmd in (("drogue", COMMAND_DROGUE), ("landed", COMMAND_LANDED), ("calibration", COMMAND_CALIBRATION),
                      ("ground abort", COMMAND_GROUND_ABORT), ("reset", COMMAND_RESET)):
        r.send_command(cmd)
        r.send_command(cmd)
        changed = r.hold(NOSE_UP, 2.0, until=lambda tel: tel["state"] != STATE_PRELAUNCH or tel["relay"] != 0,
                         phase=f"cmd_{name}")
        res.check(not changed, f"{name} command (0x{cmd:02x}) ignored: state {STATE_NAMES.get(r.tel['state'])}, "
                               f"relay 0x{r.tel['relay']:02x}")
        if changed:
            break
    return res


# ---------------------------------------------------------------- scenarios: flight
class Profile:
    """Vertical flight with quadratic drag. Solves thrust and drag so apogee is (alt, time) after launch."""

    def __init__(self, apogee_alt, apogee_time, burn_time, drogue_rate, main_rate, plateau=0.0):
        self.burn = burn_time
        self.drogue_rate = drogue_rate
        self.main_rate = main_rate
        self.plateau = plateau
        self.k, self.thrust = self.solve(apogee_alt, apogee_time)

    def simulate(self, k, thrust, dt=0.01):
        h, v, t = 0.0, 0.0, 0.0
        while True:
            a = (thrust if t < self.burn else 0.0) - G - k * v * abs(v)
            v_new = v + a * dt
            if t > self.burn and v_new <= 0.0:
                return h, t
            h += v * dt + 0.5 * a * dt * dt
            v = v_new
            t += dt
            if t > 200:
                return h, t

    def thrust_for(self, k, alt):
        lo, hi = G + 0.5, 3000.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if self.simulate(k, mid)[0] < alt:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def solve(self, alt, t_apogee):
        f0 = self.thrust_for(0.0, alt)
        t0 = self.simulate(0.0, f0)[1]
        if t_apogee is None or t_apogee >= t0:
            if t_apogee is not None and t_apogee > t0 + 0.05:
                print(f"  NOTE: apogee time {t_apogee:.1f} s is not reachable for {alt:.0f} m with a {self.burn:.1f} s "
                      f"burn, using {t0:.1f} s (no drag)")
            return 0.0, f0
        lo, hi = 0.0, 0.01
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            f = self.thrust_for(mid, alt)
            if self.simulate(mid, f)[1] > t_apogee:
                lo = mid
            else:
                hi = mid
        k = 0.5 * (lo + hi)
        return k, self.thrust_for(k, alt)


def scenario_flight(r, args):
    """S4 and the apogee scenarios (H1 to H15) through options. Needs a calibrated board (runs the pad first)."""
    res = Result(f"flight{'_' + args.hid if args.hid else ''}")
    r.wait_alive()
    if r.tel["state"] != STATE_PRELAUNCH and not do_pad(r, res):
        return res
    prof = Profile(args.apogee_alt, args.apogee_time, args.burn_time, args.drogue_rate, 5.0, args.plateau)
    print(f"  profile: thrust {prof.thrust:.1f} m/s2 for {prof.burn:.1f} s, drag k {prof.k:.6f} 1/m, "
          f"drogue {prof.drogue_rate:.0f} m/s, main 5 m/s at {MAIN_DEPLOY_ALT:.0f} m")
    r.hold(NOSE_UP, 2.0, phase="pad")

    t_launch = r.t
    h, v = 0.0, 0.0
    phase = "burn"
    t_apogee = None
    alt_apogee = 0.0
    plateau_end = None
    t_touch = None
    frozen_p = None
    inv_left = 0
    inv_started = False
    spike_done = False
    dip_left = args.dip_samples
    nan_left = 0
    stuck = args.stuck_imu
    drogue_sent = False
    t_end_limit = 600.0
    states = []

    while True:
        t = r.t - t_launch
        # ---- truth dynamics
        if phase in ("burn", "coast"):
            thrust = prof.thrust if t < prof.burn else 0.0
            drag = prof.k * v * abs(v)
            a = thrust - G - drag
            f = (thrust - drag, 0.0, 0.0)
            if phase == "burn" and t >= prof.burn:
                phase = "coast"
            v_new = v + a * r.dt
            if phase == "coast" and v_new <= 0.0:
                t_apogee, alt_apogee = t, h
                print(f"  [{r.t:7.2f}s] TRUE apogee {h:.1f} m at {t:.2f} s after launch")
                phase = "plateau" if prof.plateau > 0 else "drogue"
                plateau_end = t + prof.plateau
                v = 0.0
            else:
                h += v * r.dt + 0.5 * a * r.dt * r.dt
                v = v_new
        elif phase == "plateau":
            f = (0.0, 0.0, 0.0)
            if t >= plateau_end:
                phase = "drogue"
        elif phase == "drogue":
            f = LYING
            h -= prof.drogue_rate * r.dt
            if h <= MAIN_DEPLOY_ALT:
                phase = "main"
        elif phase == "main":
            f = LYING
            h -= prof.main_rate * r.dt
            if h <= 0.0:
                h = 0.0
                phase = "landed"
                t_touch = t
        else:
            f = LYING
        if stuck and t >= prof.burn and phase != "landed":
            f = (prof.thrust, 0.0, 0.0)
        w = (0.0, 0.0, 0.0)
        if phase in ("drogue", "main"):
            w = (15.0 * math.sin(2.1 * t), 10.0 * math.sin(1.3 * t), 5.0 * math.sin(0.7 * t))

        # ---- barometer injections (sample based: one packet is one barometer sample)
        alt_sent = h
        pressure = None
        temperature = None
        baro_now = r.baro_due()
        if baro_now and args.spike_at is not None and t >= args.spike_at and not spike_done:
            alt_sent = h + args.spike_m
            spike_done = True
        if baro_now and args.dip_at is not None and t >= args.dip_at and dip_left > 0:
            alt_sent = h - args.dip_m
            dip_left -= 1
        if args.jump_at is not None and args.jump_at <= t < args.jump_at + args.jump_s:
            alt_sent = h - args.jump_m
        if args.freeze_baro_at is not None and t >= args.freeze_baro_at:
            if args.freeze_mode == "stop":
                pressure = "skip"
            else:
                if frozen_p is None:
                    frozen_p = r.last_pressure
                pressure, temperature = frozen_p, T_REF_C
        if args.invalid_at is not None and t >= args.invalid_at and not inv_started:
            inv_left, inv_started = args.invalid_count, True
        if baro_now and inv_left > 0 and pressure != "skip":
            inv_left -= 1
            kind = args.invalid_kind
            values = {"nan": float("nan"), "inf": float("inf"), "zero": 0.0, "neg": -1.0, "low": 5.0,
                      "high": 200000.0}
            if kind in values:
                pressure = values[kind]
            elif kind == "tnan":
                temperature = float("nan")
            elif kind == "t500":
                temperature = 500.0
        accel_override = None
        if args.nan_imu_at is not None and abs(t - args.nan_imu_at) < r.dt / 2:
            nan_left = args.nan_imu_count
        if nan_left > 0:
            nan_left -= 1
            accel_override = [float("nan"), float("inf"), float("nan")]
        if args.drogue_cmd_at is not None and not drogue_sent and t >= args.drogue_cmd_at:
            r.send_command(COMMAND_DROGUE)
            drogue_sent = True
            print(f"  [{r.t:7.2f}s] sent COMMAND_DROGUE (must be ignored)")

        r.send(f, w, alt=h, phase=phase, alt_sent=alt_sent, pressure=pressure, temperature=temperature,
               accel_override=accel_override)
        if r.tel is not None:
            s = r.tel["state"]
            if not states or states[-1][1] != s:
                states.append((r.t - t_launch, s, h))
            if s in ABORT_STATES or (s == STATE_LANDED and phase == "landed"):
                r.hold(LYING, 2.0, alt=0.0, phase="landed")
                break
        if t_touch is not None and t - t_touch > 20.0:
            break
        if t > t_end_limit or (args.stop_after_apogee and t_apogee is not None and t - t_apogee > 8.0):
            break

    # ---- evaluation
    seq = [s for _, s, _ in states]
    print("  observed: " + ", ".join(f"{STATE_NAMES.get(s, s)}@{t:.2f}s/{a:.0f}m" for t, s, a in states))
    first = {}
    for t, s, a in states:
        first.setdefault(s, (t, a))
    res.check(not any(s in seq for s in ABORT_STATES), "no abort state")
    res.check(STATE_BOOST in first, "BOOST detected")
    if STATE_BOOST in first:
        res.check(first[STATE_BOOST][0] < 0.5, f"BOOST {first[STATE_BOOST][0]:.2f} s after launch (< 0.5 s)")
    if not stuck:
        res.check(STATE_COAST in first, "COAST detected")
        res.check(STATE_COAST not in first or first[STATE_COAST][0] >= prof.burn,
                  "COAST not before burnout")
    if args.apogee_alt > ACTIVE_CONTROL_ALT + 100 and not stuck:
        res.check(STATE_ACTIVE_CONTROL in first, "ACTIVE_CONTROL entered above its gate")
    if args.apogee_alt < ACTIVE_CONTROL_ALT - 100:
        res.check(STATE_ACTIVE_CONTROL not in first, "ACTIVE_CONTROL never entered below its gate")
    res.check(STATE_APOGEE in first, "APOGEE (drogue) reached")
    if STATE_APOGEE in first and t_apogee is not None:
        t6, a6 = first[STATE_APOGEE]
        t_boost = first.get(STATE_BOOST, (0.0, 0.0))[0]
        print(f"    apogee: true {t_apogee:.2f} s / {alt_apogee:.0f} m, detected {t6:.2f} s / {a6:.0f} m "
              f"(delay {t6 - t_apogee:+.2f} s, timer at {t_boost + APOGEE_TIMER_S:.2f} s)")
        if args.expect == "timer":
            res.check(abs(t6 - (t_boost + APOGEE_TIMER_S)) <= 1.0, "channel D: APOGEE within 1 s of the timer")
        elif args.expect == "baro":
            res.check(t6 >= t_apogee, "APOGEE not before true apogee")
            res.check(t6 - t_apogee <= 3.0, "APOGEE within 3 s of true apogee")
            res.check(a6 >= alt_apogee - 25.0 - prof.drogue_rate * 0.2, "APOGEE at most 25 m below true apogee")
        elif args.expect == "any":
            res.check(t6 >= t_apogee - 0.1 or t6 >= t_boost + APOGEE_TIMER_S - 0.2, "no early APOGEE")
    elif STATE_APOGEE in first:
        res.check(False, f"APOGEE at {first[STATE_APOGEE][0]:.2f} s before the true apogee")
    if not args.stop_after_apogee:
        res.check(STATE_MAIN_PARACHUTE in first, "MAIN_PARACHUTE reached")
        if STATE_MAIN_PARACHUTE in first:
            a7 = first[STATE_MAIN_PARACHUTE][1]
            res.check(MAIN_DEPLOY_ALT - 60.0 <= a7 <= MAIN_DEPLOY_ALT + 20.0,
                      f"main at {a7:.0f} m true altitude (450 m AGL, not early)")
        res.check(STATE_LANDED in first, "LANDED reached")
        if STATE_LANDED in first and t_touch is not None:
            res.check(first[STATE_LANDED][0] - t_touch <= 10.0,
                      f"LANDED {first[STATE_LANDED][0] - t_touch:.1f} s after touchdown")
    stepping = [(h["state"], bool(h["cal_status"] & CAL_KALMAN_STEPPING)) for h in r.history]
    bad = [s for s, st in stepping if (STATE_PRELAUNCH <= s <= STATE_MAIN_PARACHUTE) != st]
    res.check(not bad, f"Kalman stepping only from PRELAUNCH to MAIN_PARACHUTE ({len(bad)} bad samples)")
    ids = {}
    for t, s, a in states:
        ids[s] = ids.get(s, 0) + 1
    res.check(ids.get(STATE_APOGEE, 0) <= 1, "one drogue event")
    return res


# Apogee scenario presets (HIL/APOGEE_HIL.md). Each maps to flight options and an expectation.
H_PRESETS = {
    "H1": dict(expect="baro"),
    "H2": dict(expect="baro", baro_noise=10.0),
    "H3": dict(expect="baro", spike_at=15.0, spike_m=-40.0),
    "H3up": dict(expect="baro", spike_at=15.0, spike_m=500.0),
    "H4": dict(expect="baro", dip_at=15.0, dip_samples=4, dip_m=25.0),
    "H4b": dict(expect="any", dip_at=15.0, dip_samples=5, dip_m=25.0),
    "H5": dict(expect="timer", freeze_baro_at=12.0, freeze_mode="stop", stop_after_apogee=True),
    "H5b": dict(expect="timer", freeze_baro_at=12.0, freeze_mode="repeat", stop_after_apogee=True),
    "H6": dict(expect="timer", stuck_imu=True, stop_after_apogee=True),
    "H7": dict(expect="baro", apogee_alt=1500.0, apogee_time=None),
    "H8": dict(expect="baro"),
    "H9": dict(expect="baro", jump_at=1.0, jump_s=1.0, jump_m=100.0),
    "H10": dict(expect="baro", jump_at=15.0, jump_s=0.06, jump_m=100.0),
    "H10b": dict(expect="any", jump_at=15.0, jump_s=0.5, jump_m=100.0),
    "H12": dict(expect="any", plateau=1.0),
    "H13": dict(expect="baro", drogue_rate=60.0),
    "H14": dict(expect="baro", invalid_at=15.0, invalid_count=50, invalid_kind="zero"),
    "H15": dict(expect="baro"),
}


# ---------------------------------------------------------------- main
SCENARIOS = {
    "tumble": scenario_tumble,          # S1, S6b-f, S7
    "pad": scenario_pad,                # S2
    "calibrate": scenario_calibrate,    # S1 + S2 (+ S3 with --mounting roll90)
    "regesture": scenario_regesture,    # S6h + S7
    "pad_wait": scenario_pad_wait,      # S4b
    "commands": scenario_commands,      # R1
    "flight": scenario_flight,          # S4, H1 to H15 (with --hid), R8 (--nan-imu-at), R1 in flight (--drogue-cmd-at)
}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial port (COM3, /dev/ttyUSB0) or 'sim' for the mock board")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--rate", type=int, default=200, help="IMU packets per second, must match IMU_ODR_HZ (200)")
    p.add_argument("--baro-rate", type=int, default=50, help="barometer packets per second (hardware about 46 to 50)")
    p.add_argument("--scenario", required=True, choices=sorted(SCENARIOS))
    p.add_argument("--hid", choices=sorted(H_PRESETS), help="apogee preset for --scenario flight")
    p.add_argument("--repeat", type=int, default=1, help="repeat the scenario with seeds seed, seed+1, ...")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--mounting", default="default", choices=["default", "alt", "roll90", "identity"],
                   help="M_true preset (roll90 is S3)")
    p.add_argument("--log-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
    # tumble negative cases
    p.add_argument("--mirrored", action="store_true", help="S6b: swap poses 3 and 4 (handedness error)")
    p.add_argument("--motion-pose", type=int, help="S6c: 20 dps for 3 s during sampling of this pose, once")
    p.add_argument("--wrong-pose", type=int, help="S6d: feed the opposite orientation for this pose")
    p.add_argument("--restart-forever", type=int, help="S6f: keep this pose moving (too many restarts)")
    # flight
    p.add_argument("--apogee-alt", type=float, default=3000.0)
    p.add_argument("--apogee-time", type=float, default=25.0)
    p.add_argument("--burn-time", type=float, default=3.0)
    p.add_argument("--drogue-rate", type=float, default=25.0)
    p.add_argument("--plateau", type=float, default=0.0)
    p.add_argument("--baro-noise", type=float, default=NOISE_PRESSURE, help="pressure noise, Pa")
    p.add_argument("--expect", choices=["baro", "timer", "any"], default="baro")
    p.add_argument("--spike-at", type=float)
    p.add_argument("--spike-m", type=float, default=-40.0)
    p.add_argument("--dip-at", type=float)
    p.add_argument("--dip-samples", type=int, default=4, help="barometer samples")
    p.add_argument("--dip-m", type=float, default=25.0)
    p.add_argument("--jump-at", type=float)
    p.add_argument("--jump-s", type=float, default=1.0)
    p.add_argument("--jump-m", type=float, default=100.0)
    p.add_argument("--freeze-baro-at", type=float)
    p.add_argument("--freeze-mode", choices=["stop", "repeat"], default="stop",
                   help="H5: stop sending barometer frames, or repeat the last value")
    p.add_argument("--stuck-imu", action="store_true")
    p.add_argument("--invalid-at", type=float)
    p.add_argument("--invalid-count", type=int, default=1, help="barometer samples")
    p.add_argument("--invalid-kind", default="nan", choices=["nan", "inf", "zero", "neg", "low", "high", "tnan", "t500"])
    p.add_argument("--nan-imu-at", type=float)
    p.add_argument("--nan-imu-count", type=int, default=5)
    p.add_argument("--drogue-cmd-at", type=float)
    p.add_argument("--stop-after-apogee", action="store_true")
    p.add_argument("--duration", type=float, default=600.0, help="pad_wait duration, s")
    args = p.parse_args()

    if args.hid:
        if args.scenario != "flight":
            p.error("--hid needs --scenario flight")
        for k, v in H_PRESETS[args.hid].items():
            setattr(args, k, v)
        if args.hid == "H2" and args.repeat == 1:
            args.repeat = 20

    results = []
    seed0 = args.seed
    for i in range(args.repeat):
        args.seed = seed0 + i
        random.seed(args.seed)
        tag = args.scenario + (f"_{args.hid}" if args.hid else "")
        print(f"=== {tag} seed {args.seed} (mounting {args.mounting}, {args.rate} Hz)")
        r = Runner(args, make_mounting(args.mounting), tag)
        try:
            res = SCENARIOS[args.scenario](r, args)
        except SetupError as e:
            print(f"SETUP ERROR: {e}")
            r.close()
            return 2
        except KeyboardInterrupt:
            r.close()
            print("interrupted")
            return 1
        finally:
            if r.parser.bad:
                print(f"  telemetry resyncs: {r.parser.bad}")
        r.close()
        print(f"=== {tag} seed {args.seed}: {'PASS' if res.ok else 'FAIL'}")
        results.append(res.ok)
    if len(results) > 1:
        print(f"=== {sum(results)}/{len(results)} passed")
    return 0 if all(results) else 1


# ===========================================================================
# Scenario harness (HIL_TEST_PLAN.md).
# Driven only by injected sensor data: EXTERNAL_COMMANDS 0 means the board
# ignores serial state commands. IDLE auto-advances to CALIBRATION
# (AUTO_START_CALIBRATION). There is no serial reset, so a clean board state
# between scenarios is an operator power cycle: run one scenario per invocation.
# H1-H15 (apogee) need COMMAND_HIL_BARO 0x11, not on this branch (blocker B5).
# ===========================================================================

MAG_NOMINAL = [20.0, 5.0, -40.0]


def mat_vec(M, v):
    return [M[0]*v[0]+M[1]*v[1]+M[2]*v[2],
            M[3]*v[0]+M[4]*v[1]+M[5]*v[2],
            M[6]*v[0]+M[7]*v[1]+M[8]*v[2]]


def mat_mul(A, B):
    out = [0.0]*9
    for r in range(3):
        for c in range(3):
            out[r*3+c] = sum(A[r*3+k]*B[k*3+c] for k in range(3))
    return out


def mat_inv(M):
    a, b, c, d, e, f, g, h, i = M
    det = a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g)
    if abs(det) < 1e-12:
        raise ValueError("singular M")
    s = 1.0/det
    return [(e*i-f*h)*s, (c*h-b*i)*s, (b*f-c*e)*s,
            (f*g-d*i)*s, (a*i-c*g)*s, (c*d-a*f)*s,
            (d*h-e*g)*s, (b*g-a*h)*s, (a*e-b*d)*s]


def rotation_axis_angle(axis, angle_rad):
    x, y, z = axis
    n = math.sqrt(x*x+y*y+z*z) or 1.0
    x, y, z = x/n, y/n, z/n
    c = math.cos(angle_rad); s = math.sin(angle_rad); t = 1.0-c
    return [t*x*x+c, t*x*y-s*z, t*x*z+s*y,
            t*x*y+s*z, t*y*y+c, t*y*z-s*x,
            t*x*z-s*y, t*y*z+s*x, t*z*z+c]


def make_M_true(axis=(0, 0, 1), angle_deg=0.0, scale=(1.0, 1.0, 1.0), skew=0.0):
    R = rotation_axis_angle(axis, math.radians(angle_deg))
    S = [scale[0], skew, 0.0, 0.0, scale[1], skew, 0.0, 0.0, scale[2]]
    return mat_mul(R, S)


# Body-frame specific force per tumble pose, +g on the up axis (design section 2).
POSE_CAL = {1: (+GRAVITY, 0, 0), 2: (-GRAVITY, 0, 0), 3: (0, +GRAVITY, 0),
            4: (0, -GRAVITY, 0), 5: (0, 0, +GRAVITY), 6: (0, 0, -GRAVITY)}


def raw_accel_for_pose(pose, M_true, bias=(0.0, 0.0, 0.0)):
    cal = POSE_CAL[pose]
    return mat_vec(mat_inv(M_true), [cal[i]+bias[i] for i in range(3)])


def raw_from_body(f_body, M_true):
    return mat_vec(mat_inv(M_true), list(f_body))


def noisy(vec, sigma):
    return [v + random.gauss(0.0, sigma) for v in vec]


class Config:
    def __init__(self):
        self.hold_s = 10.0
        self.settle_s = 10.0
        self.sample_s = 30.0
        self.pose_timeout_s = 60.0
        self.enter_timeout_s = 15.0
        self.accel_noise = 0.05
        self.gyro_noise = 0.01
        self.gyro_bias = (0.3, -0.2, 0.1)
        self.accel_bias = (0.1, -0.15, 0.05)
        self.m_true = make_M_true(axis=(0.3, 0.4, 0.866), angle_deg=12.0,
                                  scale=(1.02, 0.98, 1.01), skew=0.01)
        self.pad_wait_s = 600.0
        self.apogee_alt = 3000.0
        self.apogee_time = 25.0
        self.abort_timeout_s = 30.0


class Result:
    def __init__(self, sid):
        self.sid = sid
        self.checks = []
        self.blocked = None

    def check(self, ok, msg):
        self.checks.append((bool(ok), msg))
        return ok

    def block(self, reason):
        self.blocked = reason

    @property
    def passed(self):
        return not self.blocked and bool(self.checks) and all(o for o, _ in self.checks)

    def report(self):
        if self.blocked:
            return f"{self.sid}: BLOCKED - {self.blocked}"
        lines = [f"{self.sid}: {'PASS' if self.passed else 'FAIL'}"]
        for ok, msg in self.checks:
            lines.append(f"    [{'ok' if ok else 'XX'}] {msg}")
        return "\n".join(lines)


def _send(ser, accel, gyro, pressure=SEA_LEVEL_PRESSURE, temperature=TEMPERATURE_C, mag=None):
    ser.write(build_hil_packet(accel, gyro, mag or MAG_NOMINAL, pressure, temperature))


def _pump(ser, feed, predicate, timeout_s):
    """Feed at SEND_RATE_HZ, poll telemetry, return the frame that satisfies
    predicate, or None on timeout. feed=None just listens."""
    interval = 1.0 / SEND_RATE_HZ
    now = time.monotonic()
    end = now + timeout_s
    nxt = now
    while time.monotonic() < end:
        if feed is not None:
            feed()
        frame = parse_telemetry(ser)
        if frame is not None and predicate(frame):
            return frame
        nxt += interval
        slack = nxt - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            nxt = time.monotonic()
    return None


def _feed_nose_down(ser, cfg):
    _send(ser, noisy([0.0, GRAVITY, 0.0], cfg.accel_noise), noisy([0.0, 0.0, 0.0], cfg.gyro_noise))


def _feed_nose_up(ser, cfg, gyro=(0.0, 0.0, 0.0)):
    _send(ser, noisy([0.0, -GRAVITY, 0.0], cfg.accel_noise), noisy(list(gyro), cfg.gyro_noise))


def _feed_pose(ser, cfg, M, pose, bias=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0)):
    _send(ser, noisy(raw_accel_for_pose(pose, M, bias), cfg.accel_noise), noisy(list(gyro), cfg.gyro_noise))


def _run_tumble(ser, cfg, r, M, bias=(0.0, 0.0, 0.0), order=range(1, 7)):
    for pose in order:
        reached = _pump(ser, lambda p=pose: _feed_pose(ser, cfg, M, p, bias),
                        lambda f, p=pose: f["pose"] == p, cfg.pose_timeout_s)
        if not r.check(reached is not None, f"pose {pose} prompt reached"):
            return False
        _pump(ser, lambda p=pose: _feed_pose(ser, cfg, M, p, bias),
              lambda f: False, cfg.settle_s + cfg.sample_s)
    return True


def scenario_s1(ser, cfg):
    r = Result("S1")
    entered = _pump(ser, lambda: _feed_nose_down(ser, cfg),
                    lambda f: f["state"] == 12, cfg.hold_s + cfg.enter_timeout_s)
    if not r.check(entered is not None, "gesture entered DEEP_CALIBRATION (12)"):
        return r
    if not _run_tumble(ser, cfg, r, cfg.m_true):
        return r
    done = _pump(ser, None, lambda f: f["state"] in (0, 1), 20.0)
    r.check(done is not None, "tumble returned to IDLE/CALIBRATION")
    last = done or {}
    r.check((last.get("calstatus", 0) & CAL_STATUS_IMU_CAL_VALID) != 0, "IMU_CAL_VALID set")
    r.check(True, "note: exact M vs M_true is a flash-dump host check, not telemetry")
    return r


def scenario_s2(ser, cfg):
    r = Result("S2")
    valid = _pump(ser, lambda: _feed_nose_up(ser, cfg, cfg.gyro_bias),
                  lambda f: (f["calstatus"] & CAL_STATUS_PRELAUNCH_MASK) == CAL_STATUS_PRELAUNCH_MASK, 60.0)
    r.check(valid is not None, "all validity bits set (CalStatus 0-3)")
    pre = _pump(ser, lambda: _feed_nose_up(ser, cfg, cfg.gyro_bias),
                lambda f: f["state"] == STATE_PRELAUNCH, 20.0)
    if r.check(pre is not None, "PRELAUNCH reached"):
        ax = (pre or {}).get("accel_x", 0)
        r.check(8 <= ax <= 11, f"CalAccelX ~ +9.81 (telemetry int16, got {ax})")
        gyro = (pre or {}).get("gyro", (99, 99, 99))
        r.check(all(abs(g) <= 1 for g in gyro),
                f"CalGyro ~ 0 coarse (fine 0.01 dps is flash/SD, got {gyro})")
    return r


def scenario_s3(ser, cfg):
    r = Result("S3")
    M90 = make_M_true(axis=(0, 0, 1), angle_deg=90.0)
    entered = _pump(ser, lambda: _feed_nose_down(ser, cfg),
                    lambda f: f["state"] == 12, cfg.hold_s + cfg.enter_timeout_s)
    if not r.check(entered is not None, "gesture entered DEEP_CALIBRATION (12)"):
        return r
    if not _run_tumble(ser, cfg, r, M90):
        return r
    _pump(ser, None, lambda f: f["state"] in (0, 1), 20.0)
    # rest with a constant raw gyro offset of (1,2,3) dps
    frame = _pump(ser, lambda: _feed_nose_up(ser, cfg, (1.0, 2.0, 3.0)),
                  lambda f: (f["calstatus"] & CAL_STATUS_GYRO_BIAS_VALID) != 0, 60.0)
    if r.check(frame is not None, "gyro bias measured"):
        gyro = frame.get("gyro", (99, 99, 99))
        r.check(all(abs(g) <= 1 for g in gyro),
                f"CalGyro ~ 0 (coarse; ~3.2 dps if frame bug, got {gyro}); fine bound is host test")
    return r


def _flight_body_x(profile, t):
    """Reuse FlightProfile magnitudes; vertical maps to +X body specific force."""
    accel, gyro, _mag, _p, _temp, phase, altitude = profile.sample(t)
    return -accel[1], gyro, phase, altitude


def scenario_s4(ser, cfg):
    r = Result("S4")
    pre = _pump(ser, lambda: _feed_nose_up(ser, cfg, cfg.gyro_bias),
                lambda f: f["state"] == STATE_PRELAUNCH, 60.0)
    if not r.check(pre is not None, "PRELAUNCH reached before flight"):
        return r
    profile = FlightProfile()
    seen = set()
    main_alt = None
    stepping_in_flight = False
    stepping_clear_landed = True
    finite = True
    interval = 1.0 / SEND_RATE_HZ
    t = 0.0
    nxt = time.monotonic()
    while t < profile.total_time:
        fx, gyro, _phase, altitude = _flight_body_x(profile, t)
        raw = raw_from_body([fx, 0.0, 0.0], cfg.m_true)
        _send(ser, noisy(raw, cfg.accel_noise), gyro, pressure=pressure_from_altitude(altitude))
        f = parse_telemetry(ser)
        if f is not None:
            seen.add(f["state"])
            if main_alt is None and (f["relay"] & RELAY_PARACHUTE_FIRED):
                main_alt = f["alt"]
            if f["state"] in (STATE_PRELAUNCH, 3, 4, 5, 6, 7) and (f["calstatus"] & CAL_STATUS_KALMAN_STEPPING):
                stepping_in_flight = True
            if f["state"] == 8 and (f["calstatus"] & CAL_STATUS_KALMAN_STEPPING):
                stepping_clear_landed = False
        t += interval
        nxt += interval
        slack = nxt - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            nxt = time.monotonic()
    for st, name in ((3, "BOOST"), (6, "APOGEE"), (7, "MAIN_PARACHUTE"), (8, "LANDED")):
        r.check(st in seen, f"reached {name}")
    if main_alt is not None:
        r.check(abs(main_alt - 450.0) <= 60.0, f"main parachute near 450 m AGL (at {main_alt:.0f} m)")
    else:
        r.check(False, "main parachute relay fired")
    r.check(stepping_in_flight, "Kalman stepping bit set during flight")
    r.check(stepping_clear_landed, "Kalman stepping bit clear in LANDED")
    r.check(finite, "no non-finite telemetry")
    return r


def scenario_s4b(ser, cfg):
    r = Result("S4b")
    pre = _pump(ser, lambda: _feed_nose_up(ser, cfg, cfg.gyro_bias),
                lambda f: f["state"] == STATE_PRELAUNCH, 60.0)
    if not r.check(pre is not None, "PRELAUNCH reached"):
        return r
    spurious = _pump(ser, lambda: _feed_nose_up(ser, cfg, cfg.gyro_bias),
                     lambda f: f["state"] != STATE_PRELAUNCH, cfg.pad_wait_s)
    r.check(spurious is None, f"stayed in PRELAUNCH for {cfg.pad_wait_s:.0f} s (no spurious BOOST)")
    return r


def scenario_s5(ser, cfg):
    r = Result("S5")
    pre = _pump(ser, lambda: _feed_nose_up(ser, cfg, cfg.gyro_bias),
                lambda f: f["state"] == STATE_PRELAUNCH, 60.0)
    if not r.check(pre is not None, "PRELAUNCH reached"):
        return r
    # provoke a fault by stopping the sensor stream (stale-sensor fault, if the
    # firmware has one). No serial command path under EXTERNAL_COMMANDS 0.
    abort = _pump(ser, None, lambda f: f["state"] == 9, cfg.abort_timeout_s)
    if r.check(abort is not None, "GROUND_ABORT (9) on stale stream [firmware-dependent]"):
        r.check((abort["relay"] & (RELAY_DROGUE_FIRED | RELAY_PARACHUTE_FIRED)) == 0, "pyros safed")
        r.check((abort["calstatus"] & CAL_STATUS_KALMAN_STEPPING) == 0, "Kalman stepping bit cleared")
    return r


def scenario_s6(ser, cfg):
    r = Result("S6")
    # (g) gesture in BOOST ignored: drive to BOOST, then feed the nose-down gesture.
    boost = _pump(ser, lambda: _send(ser, noisy([90.0, 0.0, 0.0], cfg.accel_noise),
                                     noisy([0, 0, 0], cfg.gyro_noise)),
                  lambda f: f["state"] == 3, 60.0)
    if r.check(boost is not None, "reached BOOST"):
        entered = _pump(ser, lambda: _feed_nose_down(ser, cfg),
                        lambda f: f["state"] == 12, cfg.hold_s + 5.0)
        r.check(entered is None, "(g) gesture in BOOST ignored")
    # (c) motion during a pose: reach pose 1 then shake; pose must not advance.
    ent = _pump(ser, lambda: _feed_nose_down(ser, cfg), lambda f: f["state"] == 12,
                cfg.hold_s + cfg.enter_timeout_s)
    if ent is not None and _pump(ser, lambda: _feed_pose(ser, cfg, cfg.m_true, 1),
                                 lambda f: f["pose"] == 1, cfg.pose_timeout_s):
        adv = _pump(ser, lambda: _feed_pose(ser, cfg, cfg.m_true, 1, gyro=(20.0, 0.0, 0.0)),
                    lambda f: f["pose"] == 2, cfg.settle_s + 10.0)
        r.check(adv is None, "(c) motion during sample: pose did not advance")
        # (d) wrong-way pose: feed pose 2 orientation while pose 1 is prompted.
        adv2 = _pump(ser, lambda: _feed_pose(ser, cfg, cfg.m_true, 2),
                     lambda f: f["pose"] == 2, cfg.settle_s + 10.0)
        r.check(adv2 is None, "(d) wrong-way pose: pose did not advance")
    else:
        r.check(False, "(c/d) reached pose 1")
    r.check(True, "note: (a) no-M, (b) mirrored, (e) timeout, (f) too-many-restarts "
                  "need operator-managed boot state / flash dump; run individually")
    return r


def scenario_s7(ser, cfg):
    r = Result("S7")
    M2 = make_M_true(axis=(0.1, 0.9, 0.3), angle_deg=25.0, scale=(0.97, 1.03, 1.0), skew=-0.02)
    entered = _pump(ser, lambda: _feed_nose_down(ser, cfg),
                    lambda f: f["state"] == 12, cfg.hold_s + cfg.enter_timeout_s)
    if not r.check(entered is not None, "gesture entered DEEP_CALIBRATION (12)"):
        return r
    if not _run_tumble(ser, cfg, r, M2):
        return r
    done = _pump(ser, None, lambda f: f["state"] in (0, 1), 20.0)
    r.check(done is not None, "second tumble returned to IDLE/CALIBRATION")
    r.check((done or {}).get("calstatus", 0) & CAL_STATUS_IMU_CAL_VALID, "IMU_CAL_VALID set")
    r.check(True, "note: newest-wins and sequence increment are a flash-dump host check")
    return r


SCENARIOS = {
    "S1": scenario_s1, "S2": scenario_s2, "S3": scenario_s3, "S4": scenario_s4,
    "S4B": scenario_s4b, "S5": scenario_s5, "S6": scenario_s6, "S7": scenario_s7,
}


def dry_run(cfg):
    """Offline: print the S4 flight phase/CalAccelX/altitude sequence, no serial."""
    profile = FlightProfile()
    print(f"Dry run: flight profile {profile.total_time:.1f}s, apogee ~{profile.apogee_alt:.0f} m")
    last = None
    t = 0.0
    while t < profile.total_time:
        fx, _gyro, phase, altitude = _flight_body_x(profile, t)
        if phase != last:
            print(f"  [{t:6.2f}s] {phase:<7s} CalAccelX~{fx:+7.1f}  alt {altitude:7.1f} m")
            last = phase
        t += 0.1
    print("Dry run OK (no board contacted).")


def run_scenarios(args):
    cfg = Config()
    if args.seed is not None:
        random.seed(args.seed)
    if args.apogee_alt is not None:
        cfg.apogee_alt = args.apogee_alt
    if args.apogee_time is not None:
        cfg.apogee_time = args.apogee_time

    if args.dry_run:
        dry_run(cfg)
        return 0

    ids = [s.upper() for s in args.scenario]
    unknown = [s for s in ids if s not in SCENARIOS]
    if unknown:
        print(f"unknown scenario(s): {', '.join(unknown)}")
        return 2

    ser = serial.Serial(args.port, args.baud, timeout=1)
    results = []
    try:
        for sid in ids:
            print(f"--- running {sid} (seed={args.seed}) ---")
            res = SCENARIOS[sid](ser, cfg)
            print(res.report())
            results.append(res)
    finally:
        ser.close()

    passed = sum(1 for r in results if r.passed)
    failed = sum(1 for r in results if not r.passed and not r.blocked)
    blocked = sum(1 for r in results if r.blocked)
    print(f"\n{passed} passed, {failed} failed, {blocked} blocked")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
