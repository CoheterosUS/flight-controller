"""Serial link to the board: send injected sensor data, read telemetry frames."""

import struct
import serial

from . import protocol as p
from .telemetry import parse_stream


def build_packet(command, payload=b""):
    return (
        bytes([p.PACKET_HEADER_LSB, p.PACKET_HEADER_MSB, command, len(payload)])
        + payload
        + bytes([p.PACKET_FOOTER])
    )


def build_hil_packet(accel, gyro, mag, pressure, temperature):
    payload = struct.pack(
        p.HIL_PAYLOAD_FORMAT,
        accel[0], accel[1], accel[2],
        gyro[0], gyro[1], gyro[2],
        mag[0], mag[1], mag[2],
        pressure,
        temperature,
    )
    return build_packet(p.COMMAND_HIL_DATA, payload)


def build_gps_packet(unix_time, milliseconds, lat_1e7, lon_1e7, alt_mm, satellites):
    payload = struct.pack(
        "<IHiiib",
        unix_time, milliseconds, lat_1e7, lon_1e7, alt_mm, satellites,
    )
    return build_packet(p.COMMAND_GPS_DATA, payload)


class Link:
    """Owns the serial port, framing and a rolling telemetry parse buffer."""

    def __init__(self, port, baud=115200, timeout=1.0):
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self._rx = bytearray()
        self._last = None

    def close(self):
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def send_hil(self, accel, gyro, mag, pressure, temperature):
        self._ser.write(build_hil_packet(accel, gyro, mag, pressure, temperature))

    def send_gps(self, unix_time, milliseconds, lat_1e7, lon_1e7, alt_mm, satellites):
        self._ser.write(
            build_gps_packet(unix_time, milliseconds, lat_1e7, lon_1e7, alt_mm, satellites)
        )

    def send_command(self, command):
        self._ser.write(build_packet(command))

    def poll(self):
        """Drain the port, return the newest decoded telemetry dict or None."""
        self._rx += self._ser.read(self._ser.in_waiting or 0)
        frame, self._rx = parse_stream(self._rx)
        if frame is not None:
            self._last = frame
        return frame

    @property
    def last(self):
        return self._last
