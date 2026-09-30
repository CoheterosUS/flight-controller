# 6-Axis Accelerometer Deep Calibration

Design for `Core/Src/States/13DeepCalibrationStateHandler.c`.

## Existing pieces

- `AccelCalibration_t` (`W25Q32JV.h`) holds `BiasX/Y/Z`, `ScaleX/Y/Z` and `Valid`.
- `FlashHeader_t` embeds `AccelCal`.
- `SystemContext_t` (`shared.h`) holds `AccelBias*`, `AccelScale*`, `AccelCalibrationValid` and `DeepCalibrationComplete`.
- `W25Q_LoadAccelCal(ctx)` copies flash values into the context (called from `1IdleStateHandler.c`).
- `W25Q_WriteAccelCal(cal)` stores a calibration in the flash header.
- `CalibrationStateHandler` enters `STATE_DEEP_CALIBRATION` when `AccelY >= DEEP_CALIBRATION_ACCEL_Y_THRESHOLD` is confirmed for `DEEP_CALIBRATION_CONFIRM_SAMPLES` samples.
- Gyro bias is handled separately by `CalibrateGyroscope`, so it is out of scope here.

## Method

Six-position static calibration. Capture the mean accelerometer reading with each axis pointing up (+g) and down (-g).

Per axis:

- `bias = (pos + neg) / 2`
- `scale = 2g / (pos - neg)`

Correction applied to raw readings:

`corrected = (raw - bias) * scale`

## Handler design

Static state in the `.c` file, same style as `Calibrations.c`:

```c
static float Sum[6][3];
static uint16_t Count;
static uint8_t Captured;
static ConfirmCounter_t Hold;
```

Orientation index: 0 = +X, 1 = -X, 2 = +Y, 3 = -Y, 4 = +Z, 5 = -Z.

Per tick in `DeepCalibrationStateHandler`:

1. Find the dominant axis and its sign. The dominant axis must satisfy `|a| >= DEEP_CALIBRATION_ACCEL_Y_THRESHOLD` and the other two axes must be small. This gives an orientation index 0..5.
2. If that orientation's bit is already set in `Captured`, ignore it.
3. Use `ConfirmCounterCheck` to require the orientation to stay steady for N samples. Reset the counter and the accumulator if the orientation changes or `|gyro|` exceeds a small motion limit.
4. Discard the first samples for settling, then accumulate the mean, as `CalibrateGyroscope` does.
5. When the sample target is reached, store the mean in `Sum[idx]` and set the bit in `Captured`.
6. When `Captured == 0x3F`:
   - compute bias and scale for each axis
   - fill an `AccelCalibration_t`
   - call `W25Q_WriteAccelCal`
   - set `ctx->AccelBias*`, `ctx->AccelScale*`, `ctx->AccelCalibrationValid = true`
   - set `ctx->DeepCalibrationComplete = true`
   - return `STATE_CALIBRATION`

Orientation is auto-detected, so faces can be presented in any order. The Y+ entry orientation from `CalibrationStateHandler` can count as the first capture.

`DeepCalibrationStateEntry` resets `Sum`, `Count`, `Captured` and `Hold`.

## Config constants to add (`configuration.h`)

- `DEEP_CALIBRATION_DISCARD_SAMPLES`
- `DEEP_CALIBRATION_SAMPLES`
- `DEEP_CALIBRATION_OFF_AXIS_MAX` (limit for the two non-dominant axes)
- `DEEP_CALIBRATION_GYRO_MOTION_MAX`
- `DEEP_CALIBRATION_TIMEOUT_MS`

## Open items

- Units. The 8.0 threshold suggests m/s², so `g = 9.80665f`. Confirm the units of `FlightData.AccelX/Y/Z`.
- `W25Q_WriteAccelCal` (`W25Q32JVHandler.c:210`) only assigns `Header.AccelCal` and sets `Valid`. Verify the header is actually programmed to flash afterwards.
- Nothing currently applies `AccelBias*` and `AccelScale*` to sensor data. Only `W25Q_LoadAccelCal` sets them. Applying them is separate work.
- Timeout behavior. Decide what happens if not all six faces are captured in time (likely return `STATE_CALIBRATION` without setting `DeepCalibrationComplete`, leaving old calibration untouched).
- Sanity check on the result. Reject the calibration if any scale is far from 1.0 or any bias is implausibly large.
