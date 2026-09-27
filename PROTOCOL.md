# Protocol Specification

Little-Endian

float: 4 bytes, IEEE 754

int8: 1 byte, Two's Complement
int16: 2 bytes, Two's Complement
int32: 4 bytes, Two's Complement

uint8: 1 byte
uint16: 2 bytes
uint32: 4 bytes

Any enum, bitmask, or structure is subject to change without notice.
Any agent that reads this file should ask any necessary clarifying questions whenever information is missing, ambiguous, incomplete, or required to proceed correctly.
Any agent that reads this file should not assume that any information is correct, complete, or up-to-date unless it is explicitly stated to be so.

## SystemState (Enum)

| Value | Name             |
|-------|------------------|
| 0     | IDLE             |
| 1     | CALIBRATION      |
| 2     | PRELAUNCH        |
| 3     | BOOST            |
| 4     | COAST            |
| 5     | ACTIVE_CONTROL   |
| 6     | APOGEE           |
| 7     | MAIN_PARACHUTE   |
| 8     | LANDED           |
| 9     | GROUND_ABORT     |
| 10    | DESCENT_ABORT    |
| 11    | ASCENT_ABORT     |
| 12    | DEEP_CALIBRATION |

## CommandType (Enum)

| Value  | Name                 | Notes                     |
|--------|----------------------|---------------------------|
| `0x00` | COMMAND_NONE         |                           |
| `0x01` | COMMAND_RESET        |                           |
| `0x02` | COMMAND_GROUND_ABORT |                           |
| `0x03` | COMMAND_CALIBRATION  |                           |
| `0x04` | COMMAND_DROGUE       |                           |
| `0x05` | COMMAND_LANDED       |                           |
| `0x10` | COMMAND_HIL_DATA     | Excluded From LastCommand |
| `0x20` | COMMAND_GPS_DATA     | Excluded From LastCommand |

## SystemFaultFlags (Bitmask)

| Bit | Flag                              |
|-----|-----------------------------------|
| 0   | BMP280_MODE_IDLE_FAILED           |
| 1   | BMP280_MODE_PERFORMANCE_FAILED    |
| 2   | BMP581_MODE_IDLE_FAILED           |
| 3   | BMP581_MODE_PERFORMANCE_FAILED    |
| 4   | IIM42653_MODE_IDLE_FAILED         |
| 5   | IIM42653_MODE_PERFORMANCE_FAILED  |
| 6   | IIS2MDCTR_MODE_IDLE_FAILED        |
| 7   | IIS2MDCTR_MODE_PERFORMANCE_FAILED |
| 8   | SD_MOUNT_FAILED                   |
| 9   | SD_OPEN_FAILED                    |

## RelayState (Bitmask)

| Bit | Flag             |
|-----|------------------|
| 0   | DROGUE_FIRED     |
| 1   | PARACHUTE_FIRED  |

## Frames and conventions

`Raw` fields are the hardware axes exactly as reported by the IMU driver. `Cal` fields are in the rocket body frame: forward, right, down. `CalAccel*` and `CalGyro*` are the calibrated body-frame values used by flight logic and the Kalman filter.

Acceleration uses the Kalman specific-force convention. An axis pointing up reads positive `g`, so nose up reads `+9.81` on `CalAccelX`. The `g` value is the Kalman constant 9.81 m/s2, not the local gravity value.

The W25Q32JV flash calibration sector and the per-flight calibration snapshot page exist in addition to the flight log. Their byte layouts are documented by WP-B in `Core/Inc/Sensors/W25Q32JV.h`.

## FlightData Record (Internal, Packed)

| Field              | Type     | Unit            | Notes                         |
|--------------------|----------|-----------------|-------------------------------|
| Sync               | uint16   |                 | `0xCAFE`                      |
| Tick               | uint32   | ms              | FreeRTOS Tick                 |
| RawAccelX          | float    | m/s2            | Hardware axes                 |
| RawAccelY          | float    | m/s2            | Hardware axes                 |
| RawAccelZ          | float    | m/s2            | Hardware axes                 |
| RawGyroX           | float    | Degrees/s       | Hardware axes                 |
| RawGyroY           | float    | Degrees/s       | Hardware axes                 |
| RawGyroZ           | float    | Degrees/s       | Hardware axes                 |
| RawMagX            | float    | Milligauss      | Hardware axes                 |
| RawMagY            | float    | Milligauss      | Hardware axes                 |
| RawMagZ            | float    | Milligauss      | Hardware axes                 |
| PressurePa         | float    | Pascals         |                               |
| TemperatureC       | float    | Celsius         |                               |
| Latitude           | int32    | Degrees x 10^7  |                               |
| Longitude          | int32    | Degrees x 10^7  |                               |
| GPSAltitude        | float    | Meters          |                               |
| UnixTime           | uint32   | Epoch Seconds   |                               |
| Milliseconds       | uint16   | 0-999           |                               |
| Satellites         | uint8    | Count           |                               |
| BarometricAltitude | float    | Meters          |                               |
| BarometricVelocity | float    | m/s             |                               |
| GPSVelocity        | float    | m/s             |                               |
| CalAccelX          | float    | m/s2            | Rocket body frame             |
| CalAccelY          | float    | m/s2            | Rocket body frame             |
| CalAccelZ          | float    | m/s2            | Rocket body frame             |
| CalGyroX           | float    | Degrees/s       | Rocket body frame             |
| CalGyroY           | float    | Degrees/s       | Rocket body frame             |
| CalGyroZ           | float    | Degrees/s       | Rocket body frame             |
| PosX               | float    | Meters          | Kalman NED                   |
| PosY               | float    | Meters          | Kalman NED                   |
| PosZ               | float    | Meters          | Kalman NED                   |
| VelX               | float    | m/s             | Kalman NED                   |
| VelY               | float    | m/s             | Kalman NED                   |
| VelZ               | float    | m/s             | Kalman NED                   |
| QuatW              | float    |                 | Kalman attitude              |
| QuatX              | float    |                 | Kalman attitude              |
| QuatY              | float    |                 | Kalman attitude              |
| QuatZ              | float    |                 | Kalman attitude              |
| PDiag[9]           | float[9] |                 | P matrix diag                |
| Flags              | uint32   | Bitmask         | SystemFaultFlags             |
| BatteryVoltage     | float    | Volts           |                               |
| State              | uint8    | SystemState     |                               |
| RelayState         | uint8    | Bitmask         | RelayState                   |
| LastCommand        | uint8    | CommandType     | Persists                     |
| CalStatus          | uint16   | Bitmask and pose | Calibration and Kalman status |
| SyncEnd            | uint8    |                 | `0xBE`                       |

## CalStatus (uint16)

| Bit or field | Name                    | Meaning                          |
|--------------|-------------------------|----------------------------------|
| 0            | IMU_CAL_VALID           | M is loaded and valid            |
| 1            | GYRO_BIAS_VALID         | Gyro bias is valid               |
| 2            | ACCEL_BIAS_VALID        | Accel bias is valid              |
| 3            | PRESSURE_REF_VALID      | Pressure reference is valid      |
| 4            | KALMAN_INITIALIZED      | Kalman filter is initialized     |
| 5            | KALMAN_STEPPING         | Kalman filter is stepping        |
| 6            | HIL_PRESEED             | HIL pre-seed is active           |
| 7            | Reserved                | Reserved                         |
| 8-10         | Tumble pose             | 0 = none, 1 to 6 = current pose  |
| 11-15        | Reserved                | Reserved                         |

## Command Frame (Structure, Packed)

Received over UART from external board (ESP32/Arduino).

| Offset | Size | Type  | Value  | Description         |
|--------|------|-------|--------|---------------------|
| 0      | 1    | uint8 | `0xFE` | Sync LSB            |
| 1      | 1    | uint8 | `0xCA` | Sync MSB            |
| 2      | 1    | uint8 | CMD    | CommandType         |
| 3      | 1    | uint8 | `0x00` | Payload Length (0)  |
| 4      | 1    | uint8 | `0xBE` | Footer              |

## GPS Data Frame (Structure, Packed)

Command `0x20` (COMMAND_GPS_DATA). Received over UART from external board (ESP32/Arduino).

| Offset | Size | Type   | Value  | Description          |
|--------|------|--------|--------|----------------------|
| 0      | 1    | uint8  | `0xFE` | Sync LSB             |
| 1      | 1    | uint8  | `0xCA` | Sync MSB             |
| 2      | 1    | uint8  | `0x20` | COMMAND_GPS_DATA     |
| 3      | 1    | uint8  | `0x13` | Payload Length (19)  |
| 4-22   |      |        |        | GPS Payload          |
| 23     | 1    | uint8  | `0xBE` | Footer               |

## GPS Payload (Structure, Packed)

| Offset | Size | Type   | Field        | Unit             |
|--------|------|--------|--------------|------------------|
| 0      | 4    | uint32 | UnixTime     | Epoch Seconds    |
| 4      | 2    | uint16 | Milliseconds | 0-999            |
| 6      | 4    | int32  | Latitude     | Degrees x 10^7   |
| 10     | 4    | int32  | Longitude    | Degrees x 10^7   |
| 14     | 4    | int32  | Altitude     | Millimeters      |
| 18     | 1    | uint8  | Satellites   | Count            |

## HIL Data Frame (Structure, Packed)

Command `0x10` (COMMAND_HIL_DATA). Received over UART from external device (Laptop, ESP32, Arduino, etc.).

| Offset | Size | Type   | Value  | Description          |
|--------|------|--------|--------|----------------------|
| 0      | 1    | uint8  | `0xFE` | Sync LSB             |
| 1      | 1    | uint8  | `0xCA` | Sync MSB             |
| 2      | 1    | uint8  | `0x10` | COMMAND_HIL_DATA     |
| 3      | 1    | uint8  | `0x2C` | Payload Length (44)  |
| 4-47   |      |        |        | HIL Payload          |
| 48     | 1    | uint8  | `0xBE` | Footer               |

## HIL Payload (Structure, Packed)

| Offset | Size | Type    | Field        | Unit       |
|--------|------|---------|--------------|------------|
| 0      | 4    | float32 | AccelX       | m/s2       |
| 4      | 4    | float32 | AccelY       | m/s2       |
| 8      | 4    | float32 | AccelZ       | m/s2       |
| 12     | 4    | float32 | GyroX        | Degrees/s  |
| 16     | 4    | float32 | GyroY        | Degrees/s  |
| 20     | 4    | float32 | GyroZ        | Degrees/s  |
| 24     | 4    | float32 | MagX         | Milligauss |
| 28     | 4    | float32 | MagY         | Milligauss |
| 32     | 4    | float32 | MagZ         | Milligauss |
| 36     | 4    | float32 | PressurePa   | Pascals    |
| 40     | 4    | float32 | TemperatureC | Celsius    |

## Wire Telemetry Packet (Structure, Packed)

The packet is 54 bytes. Accel and gyro fields are calibrated rocket body-frame values, truncated to int16.

| Offset | Size | Type   | Field              | Encoding                   |
|--------|------|--------|--------------------|----------------------------|
| 0      | 2    | uint16 | Sync               | `0xCAFE`                   |
| 2      | 4    | uint32 | Tick               | Raw                        |
| 6      | 2    | int16  | CalAccelX          | Cal body frame, truncated |
| 8      | 2    | int16  | CalAccelY          | Cal body frame, truncated |
| 10     | 2    | int16  | CalAccelZ          | Cal body frame, truncated |
| 12     | 2    | int16  | CalGyroX           | Cal body frame, truncated |
| 14     | 2    | int16  | CalGyroY           | Cal body frame, truncated |
| 16     | 2    | int16  | CalGyroZ           | Cal body frame, truncated |
| 18     | 2    | int16  | PressurePa         | /10                        |
| 20     | 1    | int8   | TemperatureC       | Truncated                  |
| 21     | 4    | int32  | Latitude           | x10^7                      |
| 25     | 4    | int32  | Longitude          | x10^7                      |
| 29     | 4    | int32  | GPSAltitude        | x100                       |
| 33     | 1    | uint8  | Satellites         | Raw                        |
| 34     | 4    | int32  | BarometricAltitude | x100                       |
| 38     | 4    | int32  | BarometricVelocity | x100                       |
| 42     | 4    | uint32 | Flags              | Bitmask                    |
| 46     | 2    | int16  | BatteryVoltage     | x10                        |
| 48     | 1    | uint8  | State              | Enum                       |
| 49     | 1    | uint8  | RelayState         | Bitmask                    |
| 50     | 1    | uint8  | LastCommand        | Enum                       |
| 51     | 2    | uint16 | CalStatus          | Bitmask and tumble pose   |
| 53     | 1    | uint8  | SyncEnd            | `0xBE`                     |

## Wire SD Log Record (Structure, Packed)

The record is 187 bytes. Raw IMU fields are hardware axes. Cal IMU fields are rocket body-frame values.

| Offset | Size | Type    | Field              | Encoding         |
|--------|------|---------|--------------------|------------------|
| 0      | 2    | uint16  | Sync               | `0xCAFE`         |
| 2      | 4    | uint32  | Tick               | Raw              |
| 6      | 4    | float32 | RawAccelX          | Hardware axes    |
| 10     | 4    | float32 | RawAccelY          | Hardware axes    |
| 14     | 4    | float32 | RawAccelZ          | Hardware axes    |
| 18     | 4    | float32 | RawGyroX           | Hardware axes    |
| 22     | 4    | float32 | RawGyroY           | Hardware axes    |
| 26     | 4    | float32 | RawGyroZ           | Hardware axes    |
| 30     | 4    | float32 | RawMagX            | Hardware axes    |
| 34     | 4    | float32 | RawMagY            | Hardware axes    |
| 38     | 4    | float32 | RawMagZ            | Hardware axes    |
| 42     | 4    | float32 | CalAccelX          | Body frame       |
| 46     | 4    | float32 | CalAccelY          | Body frame       |
| 50     | 4    | float32 | CalAccelZ          | Body frame       |
| 54     | 4    | float32 | CalGyroX           | Body frame       |
| 58     | 4    | float32 | CalGyroY           | Body frame       |
| 62     | 4    | float32 | CalGyroZ           | Body frame       |
| 66     | 4    | float32 | PressurePa         | Native           |
| 70     | 4    | float32 | TemperatureC       | Native           |
| 74     | 4    | int32   | Latitude           | x10^7            |
| 78     | 4    | int32   | Longitude          | x10^7            |
| 82     | 4    | float32 | GPSAltitude        | Native           |
| 86     | 4    | uint32  | UnixTime           | Epoch Seconds    |
| 90     | 2    | uint16  | Milliseconds       | 0-999            |
| 92     | 1    | uint8   | Satellites         | Raw              |
| 93     | 4    | float32 | BarometricAltitude | Native           |
| 97     | 4    | float32 | PosX               | Native           |
| 101    | 4    | float32 | PosY               | Native           |
| 105    | 4    | float32 | PosZ               | Native           |
| 109    | 4    | float32 | VelX               | Native           |
| 113    | 4    | float32 | VelY               | Native           |
| 117    | 4    | float32 | VelZ               | Native           |
| 121    | 4    | float32 | QuatW              | Native           |
| 125    | 4    | float32 | QuatX              | Native           |
| 129    | 4    | float32 | QuatY              | Native           |
| 133    | 4    | float32 | QuatZ              | Native           |
| 137    | 36   | float32 | PDiag[9]           | Native           |
| 173    | 4    | uint32  | Flags              | Bitmask          |
| 177    | 4    | float32 | BatteryVoltage     | Native           |
| 181    | 1    | uint8   | State              | Enum             |
| 182    | 1    | uint8   | RelayState         | Bitmask          |
| 183    | 1    | uint8   | LastCommand        | Enum             |
| 184    | 2    | uint16  | CalStatus          | Bitmask and pose |
| 186    | 1    | uint8   | SyncEnd            | `0xBE`           |

## Wire Flash Log Record (Structure, Packed)

The record is 56 bytes. Stored on W25Q32JV external flash at 10 Hz, with 4 records per 256-byte page. The 56-byte record therefore fits within the 64-byte record slot. Flight boundaries are marked by a marker record where State = `0xFF` and all sensor fields are zero. The marker is page-aligned and occupies the first 56 bytes of a 256-byte page, with the rest as `0xFF` padding.

| Offset | Size | Type    | Field      | Encoding      |
|--------|------|---------|------------|---------------|
| 0      | 2    | uint16  | Sync       | `0xCAFE`      |
| 2      | 4    | uint32  | Tick       | Raw           |
| 6      | 4    | float32 | RawAccelX  | Hardware axes |
| 10     | 4    | float32 | RawAccelY  | Hardware axes |
| 14     | 4    | float32 | RawAccelZ  | Hardware axes |
| 18     | 4    | float32 | RawGyroX   | Hardware axes |
| 22     | 4    | float32 | RawGyroY   | Hardware axes |
| 26     | 4    | float32 | RawGyroZ   | Hardware axes |
| 30     | 2    | int16   | PosX       | x1 m          |
| 32     | 2    | int16   | PosY       | x1 m          |
| 34     | 2    | int16   | PosZ       | x1 m          |
| 36     | 2    | int16   | VelX       | x10           |
| 38     | 2    | int16   | VelY       | x10           |
| 40     | 2    | int16   | VelZ       | x10           |
| 42     | 2    | int16   | QuatW      | x10000        |
| 44     | 2    | int16   | QuatX      | x10000        |
| 46     | 2    | int16   | QuatY      | x10000        |
| 48     | 2    | int16   | QuatZ      | x10000        |
| 50     | 2    | int16   | PDiag2     | x10           |
| 52     | 2    | uint16  | PressurePa | /10           |
| 54     | 1    | uint8   | State      | Enum          |
| 55     | 1    | uint8   | SyncEnd    | `0xBE`        |
