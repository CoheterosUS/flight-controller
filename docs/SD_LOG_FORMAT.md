# SD Log Binary Format

Binary telemetry log written to SD card by the flight controller. Each record is a packed, little-endian C struct (`SDLogRecord_t`) written as raw bytes with no filesystem framing — the struct **is** the record.

## Key Properties

- **Record size**: 81 bytes (fixed)
- **Byte order**: Little-endian (ARM Cortex-M7)
- **Packing**: `#pragma pack(push, 1)` — no alignment padding
- **Log rate**: ~100 Hz
- **File format**: Raw binary (`.BIN`), sequential records, no header/footer on the file itself

## Framing

Each record starts with sync word `0xCAFE` (on wire: `0xFE 0xCA` due to little-endian) and ends with footer byte `0xBE`. No length field, no checksum, no CRC.

To resync after corruption: scan for `0xFE 0xCA` bytes, attempt to parse 81 bytes, verify `SyncEnd == 0xBE`.

## Struct Definition (C)

```c
#pragma pack(push, 1)
typedef struct {
    uint16_t Sync;            // 0xCAFE
    uint32_t Tick;            // FreeRTOS tick count (ms)
    float    AccelX;          // m/s²
    float    AccelY;          // m/s²
    float    AccelZ;          // m/s²
    float    GyroX;           // degrees/s
    float    GyroY;           // degrees/s
    float    GyroZ;           // degrees/s
    float    MagX;            // milligauss
    float    MagY;            // milligauss
    float    MagZ;            // milligauss
    float    PressurePa;      // Pascals
    float    TemperatureC;    // Celsius
    int32_t  Latitude;        // degrees * 10^7
    int32_t  Longitude;       // degrees * 10^7
    float    GPSAltitude;     // meters (converted from mm)
    uint32_t UnixTime;        // seconds since epoch
    uint16_t Milliseconds;    // 0-999
    uint8_t  Satellites;      // GPS satellite count
    uint32_t Flags;           // fault bitmask (SystemFaultFlags_t)
    float    BatteryVoltage;  // Volts
    uint8_t  State;           // SystemState_t enum (0-10)
    uint8_t  RelayState;      // pyro bitmask
    uint8_t  LastCommand;     // CommandType_t, persists until next command
    uint8_t  SyncEnd;         // 0xBE
} SDLogRecord_t;
#pragma pack(pop)
```

## Field Table

| Offset | Size | Type    | Field          | Unit / Notes                        |
|--------|------|---------|----------------|-------------------------------------|
| 0      | 2    | uint16  | Sync           | Always `0xCAFE`                     |
| 2      | 4    | uint32  | Tick           | FreeRTOS tick count (ms)            |
| 6      | 4    | float32 | AccelX         | m/s²                                |
| 10     | 4    | float32 | AccelY         | m/s²                                |
| 14     | 4    | float32 | AccelZ         | m/s²                                |
| 18     | 4    | float32 | GyroX          | degrees/s                           |
| 22     | 4    | float32 | GyroY          | degrees/s                           |
| 26     | 4    | float32 | GyroZ          | degrees/s                           |
| 30     | 4    | float32 | MagX           | milligauss                          |
| 34     | 4    | float32 | MagY           | milligauss                          |
| 38     | 4    | float32 | MagZ           | milligauss                          |
| 42     | 4    | float32 | PressurePa     | Pascals                             |
| 46     | 4    | float32 | TemperatureC   | Celsius                             |
| 50     | 4    | int32   | Latitude       | degrees * 10^7                      |
| 54     | 4    | int32   | Longitude      | degrees * 10^7                      |
| 58     | 4    | float32 | GPSAltitude    | meters                              |
| 62     | 4    | uint32  | UnixTime       | seconds since epoch                 |
| 66     | 2    | uint16  | Milliseconds   | 0-999                               |
| 68     | 1    | uint8   | Satellites     | GPS satellite count                 |
| 69     | 4    | uint32  | Flags          | fault bitmask (see below)           |
| 73     | 4    | float32 | BatteryVoltage | Volts                               |
| 77     | 1    | uint8   | State          | SystemState_t enum (see below)      |
| 78     | 1    | uint8   | RelayState     | pyro bitmask (see below)            |
| 79     | 1    | uint8   | LastCommand    | CommandType_t (see below)           |
| 80     | 1    | uint8   | SyncEnd        | Always `0xBE`                       |

**Total: 81 bytes**

## Enums and Bitmasks

### SystemState_t (State field)

| Value | Name             | Description                                |
|-------|------------------|--------------------------------------------|
| 0     | IDLE             | Sensors at low power                       |
| 1     | CALIBRATION      | Sensors at max output for calibration      |
| 2     | PRELAUNCH        | SD logging initialized, waiting for launch |
| 3     | BURN             | Acceleration detected, motor burning       |
| 4     | PASSIVE_BURNOUT  | Burnout, still ascending                   |
| 5     | ACTIVE_BURNOUT   | Active control phase                       |
| 6     | APOGEE           | Apogee detected, drogue deploy             |
| 7     | PARACHUTE        | Main parachute deploy                      |
| 8     | LANDED           | On ground, GPS-only                        |
| 9     | GROUND_ABORT     | Abort on ground                            |
| 10    | DESCENT_ABORT    | Abort during descent                       |

### SystemFaultFlags_t (Flags field, bitmask)

| Bit | Name                            |
|-----|---------------------------------|
| 0   | BMP280_MODE_IDLE_FAILED         |
| 1   | BMP280_MODE_PERFORMANCE_FAILED  |
| 2   | BMP581_MODE_IDLE_FAILED         |
| 3   | BMP581_MODE_PERFORMANCE_FAILED  |
| 4   | IIM42653_MODE_IDLE_FAILED       |
| 5   | IIM42653_MODE_PERFORMANCE_FAILED|
| 6   | IIS2MDCTR_MODE_IDLE_FAILED      |
| 7   | IIS2MDCTR_MODE_PERFORMANCE_FAILED|
| 8   | SD_MOUNT_FAILED                 |
| 9   | SD_OPEN_FAILED                  |

### RelayState (bitmask)

| Bit | Name             | Notes                                  |
|-----|------------------|----------------------------------------|
| 0   | DROGUE_FIRED     | Set on PyroFire(), cleared on PyroSafe() |
| 1   | PARACHUTE_FIRED  | Set on PyroFire(), cleared on PyroSafe() |

### CommandType_t (LastCommand field)

| Value | Name               |
|-------|--------------------|
| 0x00  | COMMAND_NONE       |
| 0x01  | COMMAND_RESET      |
| 0x02  | COMMAND_GROUND_ABORT |
| 0x03  | COMMAND_CALIBRATION|
| 0x04  | COMMAND_DROGUE     |
| 0x05  | COMMAND_LANDED     |
| 0x10  | COMMAND_HIL_DATA   |
| 0x20  | COMMAND_GPS_DATA   |

## Sensor Details

| Sensor     | Interface | Fields         | Range / Scale                        |
|------------|-----------|----------------|--------------------------------------|
| IIM42653   | SPI       | Accel X/Y/Z   | 32g, 1024 LSB/g, output in m/s²     |
| IIM42653   | SPI       | Gyro X/Y/Z    | 2000 dps, 16.4 LSB/dps              |
| IIS2MDCTR  | I2C       | Mag X/Y/Z     | ±50 gauss, 1.5 mgauss/LSB           |
| BMP581     | I2C       | PressurePa     | Pascals                              |
| BMP581     | I2C       | TemperatureC   | Celsius                              |
| ZOEM8Q     | UART      | Lat/Lon/Alt    | UBX protocol, lat/lon in deg*10^7   |

## Python Decoder

`struct` format string for unpacking:

```python
import struct

# '<' = little-endian
# H=uint16, I=uint32, f=float32, i=int32, B=uint8
FORMAT = "<HI11f2ifIHBIfBBBB"
RECORD_SIZE = struct.calcsize(FORMAT)  # 81

FIELDS = [
    "Sync", "Tick",
    "AccelX", "AccelY", "AccelZ",
    "GyroX", "GyroY", "GyroZ",
    "MagX", "MagY", "MagZ",
    "PressurePa", "TemperatureC",
    "Latitude", "Longitude",
    "GPSAltitude", "UnixTime", "Milliseconds", "Satellites",
    "Flags", "BatteryVoltage",
    "State", "RelayState", "LastCommand", "SyncEnd",
]

SYNC_BYTES = b"\xFE\xCA"
SYNC_END = 0xBE
```

Resync algorithm: scan byte-by-byte for `0xFE 0xCA`, read 81 bytes, verify last byte is `0xBE`. Skip and continue on mismatch.

## Notes

- GPS coordinates stored as integers (degrees * 10^7) matching UBX native format. Divide by 10^7 for decimal degrees.
- `LastCommand` persists the most recent command received until overwritten by the next one.
- `BatteryVoltage` may read 0 if ADC not yet implemented.
- No file-level header or metadata — the `.BIN` file is just concatenated `SDLogRecord_t` records.
- SD logging starts in PRELAUNCH state and stops in LANDED state (with configurable delay).

## Difference from Telemetry Packet

The radio telemetry struct (`TelemetryPacket_t`) uses `int32_t` for most fields (floats multiplied by 100 and cast to int) to save bandwidth. SD log keeps full `float32` precision. Same framing (0xCAFE / 0xBE), different field types and layout.
