# Backlog

## Open

- [ ] **EuRoC rules: re-check every competition dependent altitude when the judges release them.** `ACTIVE_CONTROL` is a EuRoC feature. Placeholders: `COAST_ACTIVE_CONTROL_BAROM_ALT_THRESHOLD` (2000 m), the disabled 2900 m target thresholds, `APOGEE_MAIN_PARACHUTE_BAROM_ALT_THRESHOLD` (450 m AGL), `APOGEE_TIMER_MS`, and the `hil.py` profile apogee. Re-run the apogee HIL profiles after changing them.

- [x] **Fix landing check.** Barometric velocity and landed confirmation use validated new barometer samples.

- [x] **Fix barometric apogee detection.** Detector channels B and D run from BOOST, COAST and ACTIVE_CONTROL, with validated new-sample confirmations.
- [ ] **GPS velocity has the same per-loop derivative (unused, disabled)**
- [ ] **Kalman filter consumes unvalidated PressurePa (not used for any trigger)**

- [ ] **Fix the gyro bias frame mismatch (a bug the calibration rotation will expose).** `CalibrateGyroscope` (`Core/Src/Utils/Calibrations.c`) averages `FlightData.GyroX/Y/Z`, which are already rotated by `ApplyIMURotation`. `CalculateBiasedGyroscope` (`Core/Inc/Utils/Calculations.h`), called from `GetFlightData` in `Core/Src/Utils/FlightData.c`, subtracts that bias from the raw gyro values before the rotation. Bias is measured in the rotated frame and subtracted in the raw frame.
  - Harmless while the rotation is identity (`IMU_ROTATION_ENABLED 0`). With a real rotation, the bias is subtracted in the wrong frame and leaves a residual gyro offset, which feeds the Kalman attitude propagation.
  - Fix: measure the bias on the raw gyro data (before any rotation), keep subtracting it in the raw frame, then rotate. Must be in place before the deep calibration rotation is applied to the gyro.
  - Status: found by code reading (2026-09-27), not tested.
