"""Wire protocol constants shared with the firmware.

Single source of truth for the HIL harness. When WP0/WP-F finalise the
post-deep-calibration telemetry packet and the CalStatus bit layout in
PROTOCOL.md and Core/Inc/HIL/HIL.h, update THIS file to match. Everything
in harness/ and scenarios/ decodes through here.

Layout status (2026-09-27):
  - State value DEEP_CALIBRATION = 12 is planned (design D12), not yet in firmware.
  - CalStatus (uint16) is carried by overloading the BatteryVoltage slot (offset
    46) in pre-flight states, so the packet stays 52 bytes. Battery keeps that
    slot in flight states. The bit assignment below is the harness proposal.
  - Set CALSTATUS_PRESENT = True once WP-F writes CalStatus into that slot.
"""

PACKET_HEADER_LSB = 0xFE
PACKET_HEADER_MSB = 0xCA
PACKET_FOOTER = 0xBE

TELEMETRY_SYNC = 0xCAFE
TELEMETRY_SYNC_END = 0xBE

COMMAND_NONE = 0x00
COMMAND_RESET = 0x01
COMMAND_GROUND_ABORT = 0x02
COMMAND_CALIBRATION = 0x03
COMMAND_DROGUE = 0x04
COMMAND_LANDED = 0x05
COMMAND_HIL_DATA = 0x10
COMMAND_GPS_DATA = 0x20

STATE_IDLE = 0
STATE_CALIBRATION = 1
STATE_PRELAUNCH = 2
STATE_BOOST = 3
STATE_COAST = 4
STATE_ACTIVE_CONTROL = 5
STATE_APOGEE = 6
STATE_MAIN_PARACHUTE = 7
STATE_LANDED = 8
STATE_GROUND_ABORT = 9
STATE_DESCENT_ABORT = 10
STATE_DEEP_CALIBRATION = 12

STATE_NAMES = {
    STATE_IDLE: "IDLE",
    STATE_CALIBRATION: "CALIBRATION",
    STATE_PRELAUNCH: "PRELAUNCH",
    STATE_BOOST: "BOOST",
    STATE_COAST: "COAST",
    STATE_ACTIVE_CONTROL: "ACTIVE_CONTROL",
    STATE_APOGEE: "APOGEE",
    STATE_MAIN_PARACHUTE: "MAIN_PARACHUTE",
    STATE_LANDED: "LANDED",
    STATE_GROUND_ABORT: "GROUND_ABORT",
    STATE_DESCENT_ABORT: "DESCENT_ABORT",
    STATE_DEEP_CALIBRATION: "DEEP_CALIBRATION",
}


def state_name(value):
    return STATE_NAMES.get(value, f"?{value}")


# CalStatus (uint16) proposed bit layout. Low byte = validity flags, high byte
# = tumble pose. Matches the harness proposal; mirror in firmware when WP-F lands.
CALSTATUS_M_VALID = 1 << 0
CALSTATUS_GYRO_BIAS_VALID = 1 << 1
CALSTATUS_ACCEL_BIAS_VALID = 1 << 2
CALSTATUS_PRESSURE_REF_VALID = 1 << 3
CALSTATUS_KALMAN_INIT = 1 << 4
CALSTATUS_HIL_PRESEED = 1 << 5

CALSTATUS_PRELAUNCH_MASK = (
    CALSTATUS_M_VALID
    | CALSTATUS_GYRO_BIAS_VALID
    | CALSTATUS_ACCEL_BIAS_VALID
    | CALSTATUS_PRESSURE_REF_VALID
)

CALSTATUS_POSE_SHIFT = 8
CALSTATUS_POSE_MASK = 0x07 << CALSTATUS_POSE_SHIFT


def calstatus_pose(calstatus):
    return (calstatus >> CALSTATUS_POSE_SHIFT) & 0x07


def calstatus_all_valid(calstatus):
    return (calstatus & CALSTATUS_PRELAUNCH_MASK) == CALSTATUS_PRELAUNCH_MASK


# Telemetry framing. CalStatus overloads the BatteryVoltage slot (offset 46) in
# pre-flight states; packet length is unchanged. CALSTATUS_PRESENT stays False
# until WP-F actually writes CalStatus there.
TELEMETRY_PACKET_SIZE = 52
TELEMETRY_STATE_OFFSET = 48
TELEMETRY_ACCEL_X_OFFSET = 6
TELEMETRY_GYRO_X_OFFSET = 12
TELEMETRY_BARO_ALT_OFFSET = 34
TELEMETRY_BARO_VEL_OFFSET = 38
TELEMETRY_FLAGS_OFFSET = 42
TELEMETRY_BATTERY_OFFSET = 46
TELEMETRY_RELAY_OFFSET = 49
CALSTATUS_OFFSET = TELEMETRY_BATTERY_OFFSET
CALSTATUS_PRESENT = False

RELAY_DROGUE_FIRED = 1 << 0
RELAY_PARACHUTE_FIRED = 1 << 1

# HIL send payload: 11 little-endian floats (accel xyz, gyro xyz, mag xyz,
# pressure, temperature), 44 bytes.
HIL_PAYLOAD_FORMAT = "<11f"

# Nominal IMU output rate the injector feeds at. WP0 introduces IMU_ODR_HZ;
# 200 Hz is the design default (D27/D34).
IMU_ODR_HZ = 200
