# 6-Axis Accelerometer Deep Calibration

Implementation notes for `Core/Src/States/13DeepCalibrationStateHandler.c`.

## Purpose

Measure accelerometer bias and scale per axis so the Kalman filter receives corrected acceleration. Six-position static method: the board is held still on each of its six faces.

## Frame

- IMU follows the right hand rule and is mounted inverted. Upright rocket: -Y points up (toward the nose), +Y toward the tail.
- Looking along -Z toward the rocket, Y points down and X = Y cross Z.
- Accel units are m/s^2. `IMU_ROTATION_ENABLED` is 0, so `FlightData.Accel*` is the raw sensor axis.
- A resting accelerometer reads +g on the axis pointing up. The +Y face (`AccelY >= threshold`) is nose down, tail up.

## Entry

`CalibrationStateHandler` enters `STATE_DEEP_CALIBRATION` when, for `DEEP_CALIBRATION_CONFIRM_SAMPLES` consecutive samples:

- all three gyro axes are under `DEEP_CALIBRATION_GYRO_MAX_DPS` (`IsGyroscopeStill`), and
- `AccelY >= DEEP_CALIBRATION_ACCEL_THRESHOLD`.

This only happens while `WaitingDeepCalibration` is set, which is only when `DeepCalibrationComplete` is false and within `DEEP_CALIBRATION_DURATION_MS` of entering calibration.

## Capture

Faces can be captured in any order. The `Faces[]` table maps each face index to an axis and sign:

| Index | Face |
|-------|------|
| 0 | +Y |
| 1 | -Y |
| 2 | +X |
| 3 | -X |
| 4 | +Z |
| 5 | -Z |

Each tick:

1. If the gyro is not still, the detected face is none.
2. Otherwise `DetectFace` returns the face whose `axis * sign >= DEEP_CALIBRATION_ACCEL_THRESHOLD`, or none.
3. If the detected face changes, the accumulator resets.
4. A face already in `FacesCaptured` is ignored.
5. The first `DEEP_CALIBRATION_DISCARD_SAMPLES` samples are discarded so the reading can settle.
6. The next `DEEP_CALIBRATION_SAMPLES` samples are summed on all three axes.
7. The mean is stored in `FaceMean[face][3]` and the face bit is set.

There is no off-axis gate. Off-axis readings on X and Z are part of what is being measured.

## Computation and storage

When `FacesCaptured == DEEP_CALIBRATION_ALL_FACES`, `SaveCalibration`:

- computes, per axis, from the positive and negative face means, using `CalculateAccelerometerAxisCalibration` in `Calculations.c`:
  - `bias = (positive + negative) / 2`
  - `scale = 2g / (positive - negative)`
- fills `AccelCalibration_t` and calls `W25Q_WriteAccelCal`, which erases and reprograms the flash header,
- calls `W25Q_LoadAccelCal(ctx)` to copy the values into the context and set `AccelCalibrationValid`.

`DeepCalibrationComplete` is set only if the flash write succeeded. The handler then returns `STATE_CALIBRATION`.

## Configuration (`configuration.h`)

- `DEEP_CALIBRATION_ENABLED`
- `DEEP_CALIBRATION_DURATION_MS`
- `DEEP_CALIBRATION_ACCEL_THRESHOLD`
- `DEEP_CALIBRATION_CONFIRM_SAMPLES`
- `DEEP_CALIBRATION_GYRO_MAX_DPS`
- `DEEP_CALIBRATION_DISCARD_SAMPLES`
- `DEEP_CALIBRATION_SAMPLES`
- `DEEP_CALIBRATION_FACE_COUNT`
- `DEEP_CALIBRATION_ALL_FACES`
- `DEEP_CALIBRATION_AXES`

## Open items

- Apply the calibration: nothing yet computes `(raw - bias) * scale` on accel data. Gyro bias is applied in `FlightData.c`, accel is not.
- Clear `AccelCalibrationValid` on entering deep calibration so a recalibration measures raw values, not corrected ones.
- No exit from `STATE_DEEP_CALIBRATION` if the six faces are never completed.
- A failed flash write returns to `STATE_CALIBRATION` with no fault flag.
- `W25Q_LoadAccelCal` does not set `DeepCalibrationComplete`, so every boot offers deep calibration.
- No sanity check on the computed bias and scale.
- Cross-axis tilt matrix is not computed. `FaceMean` keeps all three axes per face so it can be added later: column j of M is `(mean_+j - mean_-j) / (2g)`, applied as `corrected = inverse(M) * (raw - b)`.
- Tune `DEEP_CALIBRATION_GYRO_MAX_DPS` and the sample counts on hardware.