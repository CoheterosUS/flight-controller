# Backlog

## Open

- [ ] **EuRoC rules: re-check every competition dependent altitude when the judges release them.** `ACTIVE_CONTROL` is a EuRoC feature. Placeholders: `COAST_ACTIVE_CONTROL_BAROM_ALT_THRESHOLD` (2000 m), the disabled 2900 m target thresholds, `APOGEE_MAIN_PARACHUTE_BAROM_ALT_THRESHOLD` (450 m AGL), `APOGEE_TIMER_MS`, and the `hil.py` profile apogee. Re-run the apogee HIL profiles after changing them.

- [ ] **Landing check has the same per-loop velocity problem.** `MainParachuteStateHandler` uses `fabsf(BarometricVelocity) <= 2.0`, which is exactly 0 on 4 of every 5 iterations, so the velocity half of the landed check is meaningless. Compute the velocity on new barometer samples only. Found 2026-09-27, out of scope of the apogee fix.

- [ ] **Fix barometric apogee detection (unacceptable as is).** `ACTIVE_CONTROL` goes to `APOGEE` in `Core/Src/States/6ActiveControlStateHandler.c`. The only trigger enabled in `configuration.h` is `FlightData.BarometricVelocity <= 0`, a single-sample check with no confirm counter.
  - `BarometricVelocity` is a raw one-loop derivative of the barometric altitude, recomputed every 4 ms (250 Hz loop), while the BMP581 only updates every 20 ms. On 4 of every 5 iterations the altitude is unchanged, so the velocity is exactly 0 and the condition is true. Expect a false apogee, and a drogue firing, almost immediately after entering `ACTIVE_CONTROL` (barometric altitude above 2000 m).
  - The altitude IIR filter (`CalculateFilteredAltitude`) is commented out in `GetFlightData`, so the signal is also noisy.
  - The other triggers (barometric altitude 2900 m, GPS altitude and vertical velocity, 10 s delay) are all disabled.
  - Direction: compute the velocity only on new barometer samples (or use the Kalman vertical velocity), filter it, require N consecutive samples, add a minimum time in `ACTIVE_CONTROL` and keep one independent backup trigger (altitude or delay). Verify in HIL with a noisy descent profile.
  - Status: not reproduced on hardware, found by code reading (2026-09-27).
  - UPDATED PLAN (supersedes the Direction line above): `APOGEE_DETECTION_PLAN.md`. Channels B (baro drop from peak) and D (timer) only, detector runs from `HandleState` in BOOST, COAST and ACTIVE_CONTROL so ACTIVE_CONTROL never gates apogee. HIL scenarios in `HIL/APOGEE_HIL.md`.

- [ ] **Fix the gyro bias frame mismatch (a bug the calibration rotation will expose).** `CalibrateGyroscope` (`Core/Src/Utils/Calibrations.c`) averages `FlightData.GyroX/Y/Z`, which are already rotated by `ApplyIMURotation`. `CalculateBiasedGyroscope` (`Core/Inc/Utils/Calculations.h`), called from `GetFlightData` in `Core/Src/Utils/FlightData.c`, subtracts that bias from the raw gyro values before the rotation. Bias is measured in the rotated frame and subtracted in the raw frame.
  - Harmless while the rotation is identity (`IMU_ROTATION_ENABLED 0`). With a real rotation, the bias is subtracted in the wrong frame and leaves a residual gyro offset, which feeds the Kalman attitude propagation.
  - Fix: measure the bias on the raw gyro data (before any rotation), keep subtracting it in the raw frame, then rotate. Must be in place before the deep calibration rotation is applied to the gyro.
  - Status: found by code reading (2026-09-27), not tested.
