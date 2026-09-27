"""Decode wire telemetry frames into a field dict.

Layout comes from protocol.py. When WP-F adds CalStatus, set CALSTATUS_OFFSET
and bump TELEMETRY_PACKET_SIZE there; this decoder picks it up with no change.
"""

import struct

from . import protocol as p


def _decode_frame(pkt):
    frame = {
        "accel_x": struct.unpack_from("<h", pkt, p.TELEMETRY_ACCEL_X_OFFSET)[0],
        "gyro": struct.unpack_from("<3h", pkt, p.TELEMETRY_GYRO_X_OFFSET),
        "baro_alt": struct.unpack_from("<i", pkt, p.TELEMETRY_BARO_ALT_OFFSET)[0] / 100.0,
        "baro_vel": struct.unpack_from("<i", pkt, p.TELEMETRY_BARO_VEL_OFFSET)[0] / 100.0,
        "flags": struct.unpack_from("<I", pkt, p.TELEMETRY_FLAGS_OFFSET)[0],
        "relay": pkt[p.TELEMETRY_RELAY_OFFSET],
        "state": pkt[p.TELEMETRY_STATE_OFFSET],
    }
    if p.CALSTATUS_PRESENT:
        calstatus = struct.unpack_from("<H", pkt, p.CALSTATUS_OFFSET)[0]
        frame["calstatus"] = calstatus
        frame["pose"] = p.calstatus_pose(calstatus)
    else:
        frame["calstatus"] = None
        frame["pose"] = None
    return frame


def parse_stream(rx_buf):
    """Consume whole frames from rx_buf. Return (newest_frame_or_None, remaining_buf)."""
    result = None
    size = p.TELEMETRY_PACKET_SIZE
    sync = bytes([p.PACKET_HEADER_LSB, p.PACKET_HEADER_MSB])
    while len(rx_buf) >= size:
        idx = rx_buf.find(sync)
        if idx < 0:
            rx_buf = bytearray()
            break
        if idx > 0:
            rx_buf = rx_buf[idx:]
        if len(rx_buf) < size:
            break
        if rx_buf[size - 1] != p.TELEMETRY_SYNC_END:
            rx_buf = rx_buf[1:]
            continue
        result = _decode_frame(bytes(rx_buf[:size]))
        rx_buf = rx_buf[size:]
    return result, rx_buf
