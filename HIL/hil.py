import struct
import time
import math
import argparse
import serial

PACKET_HEADER_LSB = 0xFE
PACKET_HEADER_MSB = 0xCA
PACKET_FOOTER = 0xBE

COMMAND_RESET = 0x01
COMMAND_GROUND_ABORT = 0x02
COMMAND_CALIBRATION = 0x03
COMMAND_HIL_DATA = 0x10

GRAVITY = 9.81
SEA_LEVEL_PRESSURE = 101325.0
TEMPERATURE_C = 25.0
SEND_RATE_HZ = 100

TELEMETRY_PACKET_SIZE = 52
STATE_PRELAUNCH = 2

PROFILE_THRUST_ACCEL = 90.0
PROFILE_BURN_TIME = 3.0
PROFILE_GROUND_TIME = 5.0
PROFILE_LANDED_TIME = 5.0
PROFILE_DROGUE_DESCENT_RATE = 25.0
PROFILE_MAIN_DESCENT_RATE = 5.0
PROFILE_MAIN_DEPLOY_ALT = 450.0

STATE_NAMES = {
    0: "IDLE", 1: "CALIBRATION", 2: "PRELAUNCH", 3: "BOOST",
    4: "COAST", 5: "ACTIVE_CTRL", 6: "APOGEE", 7: "MAIN_CHUTE",
    8: "LANDED", 9: "GND_ABORT", 10: "DESC_ABORT",
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
        result = {
            "accel_y": struct.unpack_from("<h", pkt, 8)[0],
            "alt": struct.unpack_from("<i", pkt, 34)[0] / 100.0,
            "vel": struct.unpack_from("<i", pkt, 38)[0] / 100.0,
            "state": pkt[48],
        }
    return result


def pressure_from_altitude(altitude_m):
    return SEA_LEVEL_PRESSURE * (1.0 - 2.25577e-5 * altitude_m) ** 5.25588


class FlightProfile:
    def __init__(self):
        self.thrust_accel = PROFILE_THRUST_ACCEL
        self.burn_time = PROFILE_BURN_TIME
        self.ground_time = PROFILE_GROUND_TIME
        self.landed_time = PROFILE_LANDED_TIME
        self.drogue_descent_rate = PROFILE_DROGUE_DESCENT_RATE
        self.main_descent_rate = PROFILE_MAIN_DESCENT_RATE
        self.main_deploy_alt = PROFILE_MAIN_DEPLOY_ALT

        self.burn_end_vel = (self.thrust_accel - GRAVITY) * self.burn_time
        self.burn_end_alt = 0.5 * (self.thrust_accel - GRAVITY) * self.burn_time ** 2
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

        t_phase = t

        if t_phase < self.ground_time:
            phase = "ground"
            accel[1] = -GRAVITY
            altitude = 0.0

        else:
            t_phase -= self.ground_time

            if t_phase < self.burn_time:
                phase = "burn"
                accel[1] = -self.thrust_accel
                altitude = 0.5 * (self.thrust_accel - GRAVITY) * t_phase ** 2
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
        return accel, gyro, mag, pressure, TEMPERATURE_C, phase, max(altitude, 0.0)


def run(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    profile = FlightProfile()
    interval = 1.0 / SEND_RATE_HZ

    print(f"Connected to {port} at {baud} baud")
    print(f"Flight profile: {profile.total_time:.1f}s total")
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
                print(f"  [{t:5.1f}s] state={sname:<12s} alt={telem['alt']:7.1f}m  vel={telem['vel']:6.1f}m/s  accelY={telem['accel_y']}m/s2")
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
            print(f"  [{t:5.1f}s] state={sname:<12s} alt={telem['alt']:7.1f}m  vel={telem['vel']:6.1f}m/s  accelY={telem['accel_y']}m/s2")
            last_print = t

        time.sleep(interval)
        t += interval

    print()
    print("Flight simulation complete.")
    ser.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HIL flight simulator for STM32 flight controller")
    parser.add_argument("--port", required=True, help="Serial port (e.g. COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()
    run(args.port, args.baud)
