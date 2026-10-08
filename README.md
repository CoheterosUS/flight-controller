- The initial deployment event shall occur at or near apogee.
- The main deployment event shall occur at an altitude no higher than 450 m AGL.

TODO:

Calibrate GPS

27-09-2026 FLIGHT COMMIT:

023892d1a2b8909c80d632380d4d8bca0fafc5ca

## Buzzer Patterns

| Count | Duration | Gap | Meaning |
|---|---|---|---|
| 5x | 300ms | 100ms | Error handler / boot error |
| 2x + 1x | 150ms + 500ms | 250ms | State machine started |
| 1x | 5ms | - | State change (debug) |
| 3x | 150ms | 150ms | Deep cal entry |
| 1-6x | 150ms | 150ms | Deep cal face captured (count = step+1) |
| 1x | 400ms | - | Deep cal timeout, skipped |
| 3x | 100ms | 150ms | Flash erase success |
| 2x | 100ms | 150ms | Flash dump success |
| 5x | 300ms | 300ms | Flash operation failed |
