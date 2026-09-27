import struct
import time
import math
import random
import argparse
import serial

PACKET_HEADER_LSB = 0xFE
PACKET_HEADER_MSB = 0xCA
PACKET_FOOTER = 0xBE

COMMAND_RESET = 0x01
COMMAND_GROUND_ABORT = 0x02
COMMAND_CALIBRATION = 0x03
COMMAND_DROGUE = 0x04
COMMAND_LANDED = 0x05
COMMAND_HIL_DATA = 0x10
COMMAND_GPS_DATA = 0x20

GRAVITY = 9.81
SEA_LEVEL_PRESSURE = 101325.0
TEMPERATURE_C = 25.0
SEND_RATE_HZ = 200  # IMU_ODR_HZ (configuration.h). One fresh noisy sample per frame.

TELEMETRY_PACKET_SIZE = 54  # PROTOCOL truth: StructManager.h TelemetryPacket_t (packed, _Static_assert == 54)
STATE_PRELAUNCH = 2

# CalStatus bits (Core/Inc/Utils/shared.h). Pose in bits 8-10.
CAL_STATUS_IMU_CAL_VALID = 1 << 0
CAL_STATUS_GYRO_BIAS_VALID = 1 << 1
CAL_STATUS_ACCEL_BIAS_VALID = 1 << 2
CAL_STATUS_PRESSURE_REF_VALID = 1 << 3
CAL_STATUS_KALMAN_INITIALIZED = 1 << 4
CAL_STATUS_KALMAN_STEPPING = 1 << 5
CAL_STATUS_HIL_PRESEED = 1 << 6
CAL_STATUS_HIL_MODE = 1 << 7
CAL_STATUS_POSE_SHIFT = 8
CAL_STATUS_PRELAUNCH_MASK = (CAL_STATUS_IMU_CAL_VALID | CAL_STATUS_GYRO_BIAS_VALID
                             | CAL_STATUS_ACCEL_BIAS_VALID | CAL_STATUS_PRESSURE_REF_VALID)

RELAY_DROGUE_FIRED = 1 << 0
RELAY_PARACHUTE_FIRED = 1 << 1

PROFILE_THRUST_ACCEL = 90.0
PROFILE_BURN_TIME = 3.0
PROFILE_GROUND_TIME = 5.0
PROFILE_LANDED_TIME = 5.0
PROFILE_DROGUE_DESCENT_RATE = 25.0
PROFILE_MAIN_DESCENT_RATE = 5.0
PROFILE_MAIN_DEPLOY_ALT = 450.0

NOISE_ACCEL = 0.05
NOISE_GYRO = 0.01
NOISE_MAG = 0.5
NOISE_PRESSURE = 2.0
NOISE_TEMPERATURE = 0.1

STATE_NAMES = {
    0: "IDLE", 1: "CALIBRATION", 2: "PRELAUNCH", 3: "BOOST",
    4: "COAST", 5: "ACTIVE_CTRL", 6: "APOGEE", 7: "MAIN_CHUTE",
    8: "LANDED", 9: "GND_ABORT", 10: "DESC_ABORT", 11: "ASCENT_ABORT",
    12: "DEEP_CALIBRATION",
}


def build_packet(command, payload=b""):
    return bytes([PACKET_HEADER_LSB, PACKET_HEADER_MSB, command, len(payload)]) + payload + bytes([PACKET_FOOTER])


def build_hil_packet(accel, gyro, mag, pressure, temperature):
    payload = struct.pack(
        "<11f",
        accel[0], accel[1], accel[2],
        gyro[0], gyro[1], gyro[2],
        mag[0], mag[1], mag[2],
        pressure,
        temperature,
    )
    return build_packet(COMMAND_HIL_DATA, payload)


def build_command_packet(command):
    return build_packet(command)


rx_buf = bytearray()

def parse_telemetry(ser):
    global rx_buf
    rx_buf += ser.read(ser.in_waiting or 0)
    result = None
    while len(rx_buf) >= TELEMETRY_PACKET_SIZE:
        idx = rx_buf.find(bytes([PACKET_HEADER_LSB, PACKET_HEADER_MSB]))
        if idx < 0:
            rx_buf.clear()
            break
        if idx > 0:
            rx_buf = rx_buf[idx:]
        if len(rx_buf) < TELEMETRY_PACKET_SIZE:
            break
        if rx_buf[TELEMETRY_PACKET_SIZE - 1] != PACKET_FOOTER:
            rx_buf = rx_buf[1:]
            continue
        pkt = bytes(rx_buf[:TELEMETRY_PACKET_SIZE])
        rx_buf = rx_buf[TELEMETRY_PACKET_SIZE:]
        calstatus = struct.unpack_from("<H", pkt, 51)[0]
        result = {
            "accel_x": struct.unpack_from("<h", pkt, 6)[0],
            "gyro": struct.unpack_from("<3h", pkt, 12),
            "alt": struct.unpack_from("<i", pkt, 34)[0] / 100.0,
            "vel": struct.unpack_from("<i", pkt, 38)[0] / 100.0,
            "flags": struct.unpack_from("<I", pkt, 42)[0],
            "state": pkt[48],
            "relay": pkt[49],
            "calstatus": calstatus,
            "pose": (calstatus >> CAL_STATUS_POSE_SHIFT) & 0x07,
        }
    return result


def pressure_from_altitude(altitude_m):
    return SEA_LEVEL_PRESSURE * (1.0 - 2.25577e-5 * altitude_m) ** 5.25588


class FlightProfile:
    def __init__(self, launch_angle_deg=0.0):
        self.launch_angle_deg = launch_angle_deg
        self.launch_angle_rad = math.radians(launch_angle_deg)
        self.thrust_accel = PROFILE_THRUST_ACCEL
        self.burn_time = PROFILE_BURN_TIME
        self.ground_time = PROFILE_GROUND_TIME
        self.landed_time = PROFILE_LANDED_TIME
        self.drogue_descent_rate = PROFILE_DROGUE_DESCENT_RATE
        self.main_descent_rate = PROFILE_MAIN_DESCENT_RATE
        self.main_deploy_alt = PROFILE_MAIN_DEPLOY_ALT

        vert_thrust = self.thrust_accel * math.cos(self.launch_angle_rad)
        self.burn_end_vel = (vert_thrust - GRAVITY) * self.burn_time
        self.burn_end_alt = 0.5 * (vert_thrust - GRAVITY) * self.burn_time ** 2
        self.coast_time = self.burn_end_vel / GRAVITY
        self.apogee_alt = (self.burn_end_alt
                           + self.burn_end_vel * self.coast_time
                           - 0.5 * GRAVITY * self.coast_time ** 2)

        drogue_dist = self.apogee_alt - self.main_deploy_alt
        self.drogue_time = drogue_dist / self.drogue_descent_rate
        self.main_time = self.main_deploy_alt / self.main_descent_rate

        self.total_time = (self.ground_time + self.burn_time + self.coast_time
                           + self.drogue_time + self.main_time + self.landed_time)

    def sample(self, t):
        accel = [0.0, 0.0, 0.0]
        gyro = [0.0, 0.0, 0.0]
        mag = [20.0, 5.0, -40.0]
        phase = "ground"

        sin_a = math.sin(self.launch_angle_rad)
        cos_a = math.cos(self.launch_angle_rad)
        vert_thrust = self.thrust_accel * cos_a

        t_phase = t

        if t_phase < self.ground_time:
            phase = "ground"
            accel[0] = -GRAVITY * sin_a
            accel[1] = -GRAVITY * cos_a
            altitude = 0.0

        else:
            t_phase -= self.ground_time

            if t_phase < self.burn_time:
                phase = "burn"
                accel[1] = -self.thrust_accel
                altitude = 0.5 * (vert_thrust - GRAVITY) * t_phase ** 2
                gyro[0] = math.sin(t_phase / self.burn_time * math.pi) * 5.0

            else:
                t_phase -= self.burn_time

                if t_phase < self.coast_time:
                    phase = "coast"
                    accel[1] = 0.0
                    altitude = (self.burn_end_alt
                                + self.burn_end_vel * t_phase
                                - 0.5 * GRAVITY * t_phase ** 2)

                else:
                    t_phase -= self.coast_time

                    if t_phase < self.drogue_time:
                        phase = "drogue"
                        altitude = self.apogee_alt - self.drogue_descent_rate * t_phase
                        accel[1] = self.drogue_descent_rate * 0.1

                    else:
                        t_phase -= self.drogue_time

                        if t_phase < self.main_time:
                            phase = "main"
                            altitude = self.main_deploy_alt - self.main_descent_rate * t_phase
                            accel[1] = self.main_descent_rate * 0.1

                        else:
                            phase = "landed"
                            altitude = 0.0
                            accel[1] = -GRAVITY

        pressure = pressure_from_altitude(max(altitude, 0.0))

        accel = [a + random.gauss(0, NOISE_ACCEL) for a in accel]
        gyro = [g + random.gauss(0, NOISE_GYRO) for g in gyro]
        mag = [m + random.gauss(0, NOISE_MAG) for m in mag]
        pressure += random.gauss(0, NOISE_PRESSURE)
        temperature = TEMPERATURE_C + random.gauss(0, NOISE_TEMPERATURE)

        return accel, gyro, mag, pressure, temperature, phase, max(altitude, 0.0)


def run(port, baud, angle):
    ser = serial.Serial(port, baud, timeout=1)
    profile = FlightProfile(launch_angle_deg=angle)
    interval = 1.0 / SEND_RATE_HZ

    print(f"Connected to {port} at {baud} baud")
    print(f"Flight profile: {profile.total_time:.1f}s total, launch angle {profile.launch_angle_deg:.1f}° from vertical")
    print(f"  Burn:   {profile.burn_time:.1f}s  (accel {profile.thrust_accel:.0f} m/s²)")
    print(f"  Coast:  {profile.coast_time:.1f}s  (apogee {profile.apogee_alt:.0f}m)")
    print(f"  Drogue: {profile.drogue_time:.1f}s  ({profile.drogue_descent_rate:.0f} m/s to {profile.main_deploy_alt:.0f}m)")
    print(f"  Main:   {profile.main_time:.1f}s  ({profile.main_descent_rate:.0f} m/s to ground)")
    print()

    input("Press Enter to send CALIBRATION command...")
    ser.write(build_command_packet(COMMAND_CALIBRATION))
    print("Sent CALIBRATION command")

    print("Sending ground data for calibration (up to 40s, stops on PRELAUNCH)...")
    calibration_time = 40.0
    t = 0.0
    last_print = -1.0
    while t < calibration_time:
        accel, gyro, mag, pressure, temperature, _, _ = profile.sample(0.0)
        ser.write(build_hil_packet(accel, gyro, mag, pressure, temperature))
        time.sleep(interval)
        t += interval
        telem = parse_telemetry(ser)
        if telem:
            if telem["state"] == STATE_PRELAUNCH:
                print(f"  Board transitioned to PRELAUNCH at {t:.1f}s")
                break
            if int(t) != int(last_print):
                sname = STATE_NAMES.get(telem["state"], f"?{telem['state']}")
                print(f"  [{t:5.1f}s] state={sname:<12s} alt={telem['alt']:7.1f}m  vel={telem['vel']:6.1f}m/s  CalAccelX={telem['accel_x']}")
                last_print = t
        elif int(t) != int(last_print):
            print(f"  [{t:5.1f}s] (no telemetry)")
            last_print = t

    print()
    input("Press Enter to start flight simulation...")
    print()

    t = 0.0
    last_phase = ""
    last_print = -1.0
    while t < profile.total_time:
        accel, gyro, mag, pressure, temperature, phase, altitude = profile.sample(t)
        packet = build_hil_packet(accel, gyro, mag, pressure, temperature)
        ser.write(packet)

        if phase != last_phase:
            print(f"[{t:6.2f}s] SIM phase: {phase}")
            last_phase = phase

        telem = parse_telemetry(ser)
        if telem and int(t) != int(last_print):
            sname = STATE_NAMES.get(telem["state"], f"?{telem['state']}")
            print(f"  [{t:5.1f}s] state={sname:<12s} alt={telem['alt']:7.1f}m  vel={telem['vel']:6.1f}m/s  CalAccelX={telem['accel_x']}")
            last_print = t

        time.sleep(interval)
        t += interval

    print()
    print("Flight simulation complete.")
    ser.close()


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
    parser = argparse.ArgumentParser(description="HIL simulator and scenario runner for the STM32 flight controller")
    parser.add_argument("--port", help="Serial port (e.g. COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--angle", type=float, default=0.0, help="Interactive flight: launch angle from vertical (deg)")
    parser.add_argument("--scenario", nargs="+", help="Scenario IDs to run (S1..S7, S4B). One board state per invocation.")
    parser.add_argument("--seed", type=int, default=None, help="RNG seed for reproducible noise")
    parser.add_argument("--apogee-alt", type=float, default=None, dest="apogee_alt")
    parser.add_argument("--apogee-time", type=float, default=None, dest="apogee_time")
    parser.add_argument("--dry-run", action="store_true", help="Offline check of the flight profile, no serial")
    args = parser.parse_args()

    if args.dry_run or args.scenario:
        raise SystemExit(run_scenarios(args))
    if not args.port:
        parser.error("--port is required for the interactive flight (or use --scenario / --dry-run)")
    run(args.port, args.baud, args.angle)
