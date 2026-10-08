- The initial deployment event shall occur at or near apogee.
- The main deployment event shall occur at an altitude no higher than 450 m AGL.

TODO:

Calibrate GPS

27-09-2026 FLIGHT COMMIT:

023892d1a2b8909c80d632380d4d8bca0fafc5ca

## Buzzer Patterns

| Pattern | Meaning |
|---|---|
| 5x 200ms (boot) | Error handler / boot error |
| 2x 100ms | State machine started |
| 1x 5ms | State change (debug) |
| 3x 80ms | Deep cal entry |
| 1-6x 80ms | Deep cal face captured (count = step+1) |
| 1x 80ms | Deep cal timeout, skipped |
| 3x 100ms | Flash erase success |
| 2x 100ms | Flash dump success |
| 5x 100ms | Flash operation failed |
