# 6-Axis Accelerometer Deep Calibration

Implementation notes for `Core/Src/States/13DeepCalibrationStateHandler.c`.

## Purpose

Measure accelerometer bias, scale, and cross-axis coupling so the Kalman filter receives corrected acceleration. Six-position static method: the board is held still on each of its six faces in sequence.

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

Faces are captured in fixed sequential order (step 0 through 5). The user must rotate the board, hold still, let it sample, then rotate again. Between faces the state machine requires motion before it will accept stillness for the next face.

Phase machine per face:

1. `PHASE_WAITING_MOTION` -- waits for the board to move (gyro not still). Prevents re-sampling the same orientation.
2. `PHASE_SETTLING` -- waits for stillness after motion.
3. `PHASE_DISCARDING` -- discards `DEEP_CALIBRATION_DISCARD_SAMPLES` while still, letting the reading settle. Resets to SETTLING if motion detected.
4. `PHASE_SAMPLING` -- collects `DEEP_CALIBRATION_SAMPLES` readings. Resets to SETTLING if motion detected.

Readings are converted from m/s^2 back to raw LSB via `CalculateAccelerationLSB` and stored as `int16_t` in `CalibrationMatrix[step][sample][axis]`.

After sampling completes for a face, the face bit is set in `DeepCalFacesCaptured`, the timeout resets, and a buzzer beep sequence indicates progress.

On entry, `DeepCalibrationStateEntry` clears `AccelCalibrationValid` so readings are raw during calibration.

## Face order assumption

The Y vector in `six_point_cal` assumes this face order:

| Step | Expected orientation |
|------|---------------------|
| 0 | +X up |
| 1 | -X up |
| 2 | +Y up |
| 3 | -Y up |
| 4 | +Z up |
| 5 | -Z up |

The code does not detect which face is up. The user must follow this order.

## Math (`six_point_cal` in `Core/Src/Kalman/Cal.c`)

Least-squares solve using CMSIS-DSP matrix operations.

**Inputs:**
- W = `[raw_x, raw_y, raw_z, 1]` for all 6*N samples (N = `DEEP_CALIBRATION_SAMPLES` = 500). Size 3000 x 4.
- Y = known gravity reference vectors. Size 3000 x 3. Each block of N rows gets the expected gravity for that face: `[+9.8,0,0]`, `[-9.8,0,0]`, `[0,+9.8,0]`, `[0,-9.8,0]`, `[0,0,+9.8]`, `[0,0,-9.8]`.

**Solve:**
```
X = (W^T W)^-1 W^T Y
```

X is 4 x 3. The top 3x3 rows encode scale, rotation, and cross-axis correction. Row 4 (index 9-11) is the bias vector.

**Post-processing:**
- M = transpose of top 3x3 of X. This is the full correction matrix (rotation + scale + cross-axis).
- A_m = M with each column normalized to unit length. This is the pure rotation matrix (IMU frame to body frame).

## Application (`FlightData.c`)

When `AccelCalibrationValid` is true:

**Accel** (`CalculateCalibratedAccel`):
```
lsb = raw_m_s2 / ACCEL_SCALE
BodyAccel = M * lsb
```
Converts m/s^2 back to LSB, then applies M. Output goes to `BodyAccel{X,Y,Z}`.

**Gyro** (`CalculateRotatedVector`):
```
BodyGyro = A_m * biased_gyro
```
Rotates bias-corrected gyro into body frame using the normalized rotation matrix.

When invalid, `Body*` fields pass through raw sensor values.

## Storage

Three outputs saved to flash via `W25Q_WriteAccelCal` and loaded into context:
- `AccelM` -- 3x3 correction matrix (M)
- `AccelA_m` -- 3x3 rotation matrix (normalized M)
- `AccelBias` -- 3-element bias from row 4 of X

## State entry and exit

On entry, `DeepCalibrationStateEntry` clears `AccelCalibrationValid`, `DeepCalFacesCaptured`, `DeepCalCurrentFace`, and all accumulators. Phase starts as `PHASE_WAITING_MOTION`. Clearing the calibration flag means faces are measured raw.

Exits:

- **Completion:** all six faces captured, `RunSixPointCal` computes calibration, values written to flash and context, returns `STATE_CALIBRATION`.
- **Timeout:** if `GetStateElapsedMs(Context, STATE_DEEP_CALIBRATION) >= DEEP_CALIBRATION_TIMEOUT_MS`, calls `W25Q_LoadAccelCal(Context)` to restore stored calibration and returns `STATE_CALIBRATION`. Timeout resets after each captured face, so it is per-face, not total.
- **Commands:** `HandleCommand` can force other states (reset, ground abort, drogue, landed). Reset goes through `IdleStateEntry`, which reloads the calibration. The others do not reload, so the flag stays false until the next IDLE.

If the board is still on +Y after a timeout, `CalibrationStateEntry` re-arms the wait window and the entry check can fire again.

## Configuration (`configuration.h`)

- `DEEP_CALIBRATION_ENABLED`
- `DEEP_CALIBRATION_DURATION_MS`
- `DEEP_CALIBRATION_ACCEL_THRESHOLD`
- `DEEP_CALIBRATION_CONFIRM_SAMPLES`
- `DEEP_CALIBRATION_GYRO_MAX_DPS`
- `DEEP_CALIBRATION_DISCARD_SAMPLES`
- `DEEP_CALIBRATION_SAMPLES`
- `DEEP_CALIBRATION_TIMEOUT_MS`
- `DEEP_CALIBRATION_FACE_COUNT`
- `DEEP_CALIBRATION_ALL_FACES`
- `DEEP_CALIBRATION_AXES`

## Open items

- `AccelBias` is computed and stored but never applied. `CalculateCalibratedAccel` does `M * lsb` with no bias subtraction. Either bias needs to be subtracted before the matrix multiply, or M needs to be augmented to absorb it.
- A failed flash write returns to `STATE_CALIBRATION` with no fault flag. `W25Q_WriteAccelCal` sets `Header.AccelCal` in RAM before the flash write, so a later IDLE reload could load values that never reached flash.
- `AccelCalibrationValid` is a plain `bool`, not `volatile`. This matters once another task reads it.
- `W25Q_LoadAccelCal` does not set `DeepCalibrationComplete`, so every boot offers deep calibration.
- No sanity check on the computed bias, scale, or matrix condition.
- `N` is hardcoded to 500 in `Cal.h`, must match `DEEP_CALIBRATION_SAMPLES` in `configuration.h`.
- `six_point_cal` uses 9.8 instead of a defined constant for g.
- `CalibrationMatrix` stores int16_t, so LSB values outside int16 range would overflow.
- Tune `DEEP_CALIBRATION_GYRO_MAX_DPS`, the sample counts and `DEEP_CALIBRATION_TIMEOUT_MS` on hardware.
