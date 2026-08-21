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


def pressure_from_altitude(altitude_m):
    return SEA_LEVEL_PRESSURE * (1.0 - 2.25577e-5 * altitude_m) ** 5.25588


class FlightProfile:
    def __init__(self):
        self.thrust_accel = 40.0
        self.burn_time = 3.0
        self.ground_time = 5.0
        self.landed_time = 5.0
        self.drogue_descent_rate = 15.0
        self.main_descent_rate = 5.0
        self.main_deploy_alt = 500.0

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
                accel[1] = self.thrust_accel - GRAVITY
                altitude = 0.5 * (self.thrust_accel - GRAVITY) * t_phase ** 2
                gyro[0] = math.sin(t_phase / self.burn_time * math.pi) * 5.0

            else:
                t_phase -= self.burn_time

                if t_phase < self.coast_time:
                    phase = "coast"
                    accel[1] = -GRAVITY
                    altitude = (self.burn_end_alt
                                + self.burn_end_vel * t_phase
                                - 0.5 * GRAVITY * t_phase ** 2)

                else:
                    t_phase -= self.coast_time

                    if t_phase < self.drogue_time:
                        phase = "drogue"
                        altitude = self.apogee_alt - self.drogue_descent_rate * t_phase
                        accel[1] = -GRAVITY

                    else:
                        t_phase -= self.drogue_time

                        if t_phase < self.main_time:
                            phase = "main"
                            altitude = self.main_deploy_alt - self.main_descent_rate * t_phase
                            accel[1] = -GRAVITY

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

    print("Sending ground data for calibration (40s)...")
    calibration_time = 40.0
    t = 0.0
    while t < calibration_time:
        accel, gyro, mag, pressure, temperature, _, _ = profile.sample(0.0)
        ser.write(build_hil_packet(accel, gyro, mag, pressure, temperature))
        time.sleep(interval)
        t += interval
        if int(t) != int(t - interval):
            print(f"  Calibrating... {t:.0f}s / {calibration_time:.0f}s")

    print()
    input("Press Enter to start flight simulation...")
    print()

    t = 0.0
    last_phase = ""
    while t < profile.total_time:
        accel, gyro, mag, pressure, temperature, phase, altitude = profile.sample(t)
        packet = build_hil_packet(accel, gyro, mag, pressure, temperature)
        ser.write(packet)

        if phase != last_phase:
            print(f"[{t:6.2f}s] Phase: {phase:<12s}  Altitude: {altitude:8.1f}m  AccelY: {accel[1]:7.2f} m/s²")
            last_phase = phase

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
