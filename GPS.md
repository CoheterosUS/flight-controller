# GPS Packet Structure

GPS data is received over UART from an external board (ESP32/Arduino) connected to a ZOEM8Q module. Data is wrapped in the standard protocol framing.

## Protocol Frame

| Byte | Value | Description |
|------|-------|-------------|
| 0 | `0xFE` | Header LSB |
| 1 | `0xCA` | Header MSB |
| 2 | `0x20` | Command (COMMAND_GPS_DATA) |
| 3 | `0x13` | Payload length (19 bytes) |
| 4-22 | ... | Payload (see below) |
| 23 | `0xBE` | Footer |

## Payload (19 bytes, little-endian)

| Offset | Size | Type | Field | Unit |
|--------|------|------|-------|------|
| 0 | 4 | uint32 | Unix time | seconds |
| 4 | 2 | uint16 | Milliseconds | ms (0-999) |
| 6 | 4 | int32 | Latitude | degrees × 10^7 |
| 10 | 4 | int32 | Longitude | degrees × 10^7 |
| 14 | 4 | int32 | Altitude | millimeters |
| 18 | 1 | uint8 | Satellites | count |
