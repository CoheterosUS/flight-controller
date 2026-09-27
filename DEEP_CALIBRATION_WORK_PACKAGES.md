# Deep calibration: work packages for concurrent work

Read `DEEP_CALIBRATION_DESIGN.md` first. Decision IDs (D1, D2, ...) refer to that document. Work happens on branch `kalman-filter`. The barometric apogee fix is a separate branch and session (`baro-apogee-fix`) and is not part of this plan.

## 1. Rules for working concurrently

1. WP0 lands first, alone. It contains every shared rename, type, field and macro so the other packages only fill in their own files.
2. Each package owns its files (section 3). Do not edit a file owned by another package. If you need a change there, ask its owner or add it to the interface contract.
3. Shared headers (`shared.h`, `configuration.h`) are edited only by WP0. Other packages only change values inside the labeled macro block WP0 created for them.
4. Code against the interface contracts (section 4), with stubs, so packages do not wait for each other.
5. Commit per package, on its own short-lived branch off `kalman-filter`, and merge in the order in section 5. Run the full checks once per batch.
6. Every package ships its own tests (host tests where the code is pure, HIL scenarios where it is not).

## 2. Packages

### WP0: Foundation (first, single owner)

Goal: make the tree compile with the new names and types, with stubs, so WP-A to WP-H can start.
- Rename per D28: `FlightData_t` fields (`RawAccel*`, `RawGyro*`, `RawMag*`; `CalAccel*` and `CalGyro*` stay), record fields in `SDLogRecord_t`, `FlashLogRecord_t`, `TelemetryPacket_t` (names and meaning per D31 and D32, layout unchanged for now), `SystemContext_t` (`GyroBiasRaw*`, `AccelBiasCal*`).
- Point every consumer at the right field. The compiler finds them all. The behaviour stays unchanged: the state handlers keep reading the same physical axis (now named `RawAccelY`). The switch to `CalAccelX` with the sign flip (D30) belongs to WP-E, because it only makes sense once `Cal` is computed from `M`.
- Add to `SystemContext_t`: `ImuCalibration_t ImuCal`, `float AccelBiasCalX/Y/Z`, `bool AccelBiasCalValid`, `uint16_t CalStatus` (the renamed `GyroBiasRawX/Y/Z` scalars stay scalars).
- Add `STATE_DEEP_CALIBRATION` (12), `StateHandlers.h` declarations, `13DeepCalibrationStateHandler.c` with an empty handler, and the cases in `OnStateEntry` and `HandleState`.
- Add every new macro (section 6) to `configuration.h` with defaults, in one block per package.
- Update `PROTOCOL.md` for the state value only.
- Definition of done: builds, behaviour unchanged except the renames (the compile check passes at the same level as the baseline).

### WP-A: Tumble math module (independent)

Files: `Core/Src/Utils/ImuTumbleCal.c`, `Core/Inc/Utils/ImuTumbleCal.h`, the CMSIS-DSP sources for QR (`arm_mat_qr_f32.c` and its Householder helper, `ARM_DSP_ATTRIBUTE` removed as in `Core/Src/Kalman_C/readme.txt`), host tests.
- Pure C, no HAL, no RTOS. Implements D1, D4 (pose check), D6 (solve, QR, sign normalization, rejection).
- Host tests: synthetic `M_true` (rotation, scale, skew, offsets, noise) recovers `M` and `Q`; mirrored pose order gives `det(Q) = -1` and is rejected; a wrong-way pose is flagged; a singular case does not crash.

### WP-B: Flash storage of M (independent)

Files: `Core/Src/Sensors/W25Q32JV/*`, `Core/Inc/Sensors/W25Q32JV.h`.
- Implements D16 to D18: reserved last sector at `0x3FF000`, append-only records, `W25Q_LOG_END`, `W25Q_EraseAll` preserves the record, `W25Q_DumpToSD` skips it, a snapshot-page writer for D31.
- Must never lose the previous record on a power loss during an append (commit fields last).
- Tests: record round trip, torn-record fallback, sector-full erase and rewrite, erase-all preservation.

### WP-C: Buzzer task (independent)

Files: `Core/Src/Sensors/Buzzer.c`, new `Core/Src/Tasks/BuzzerTask.c` and header, `main.c` (one line to create the task).
- Non-blocking `Buzzer_Play(pattern)` through a task notification. Patterns per D13 and D14. Stack macro `STACK_SIZE_BUZZER`, lowest priority.
- Existing `Buzzer_Beep` and `Buzzer_Beep_Counter` stay.

### WP-D: Deep calibration state (depends on interfaces A, B, C)

Files: `Core/Src/States/13DeepCalibrationStateHandler.c`, `Core/Src/Managers/StateManager.c`, `Core/Src/States/1IdleStateHandler.c`, `Core/Src/States/9LandedStateHandler.c` if needed.
- Implements D8 to D12: gesture detector (tick-based, D9), the non-blocking pose sub-state machine (prompt, settle, check, sample, restart, timeout), calling the WP-A module, storing through WP-B, prompting through WP-C.
- `HandleSensors` and `OnStateEntry` changes, including the `SensorsIdleFinished` fix (D35).
- Check the state machine task stack high-water mark. The solve uses 4x4 doubles and CMSIS QR buffers; the Kalman already needs a large stack. Raise `STACK_SIZE_STATE_MACHINE` if needed.

### WP-E: Calibrated data flow, pad calibration and Kalman gating (depends on interface B for load)

Files: `Core/Src/Utils/FlightData.c` and header, `Core/Src/Utils/Calibrations.c` and header, `Core/Src/Utils/Calculations.c` and header (bias and rotation helpers only), `Core/Src/States/2CalibrationStateHandler.c`, `Core/Src/States/3PrelaunchStateHandler.c`, the abort state handlers, `main.c` (IMU timer line), `Core/Src/Sensors/IIM42653/IIM42653Handler.c` (ODR constant).
- D19 to D23: `Cal` fields every loop (`ImuApplyCalibration`), gyro bias on raw data (the bug fix), accel bias in the body frame, tilt and stillness gates, load `M` at boot, the PRELAUNCH gate, retire `ApplyIMURotation`.
- D30: switch the boost and burnout thresholds (states 3 and 4) from raw Y to `CalAccelX` with the sign flip, in the same change that makes `Cal` real.
- D24 to D27: Kalman initialised at the end of CALIBRATION, stepping from PRELAUNCH, stopping in abort states, `KALMAN_DT` from `IMU_ODR_HZ`.
- D34: `IMU_ODR_HZ` selects the ODR register value and the IMU timer period.
- Do not touch `CalculateBarometricVerticalVelocity` behaviour (the apogee session owns it).
- Regression test (HIL): a 90 deg rotation about Z with a constant raw gyro offset (1, 2, 3) dps at rest must give `CalGyro` within 0.01 dps of zero. Before the fix the residual is about 3.2 dps.

### WP-F: Logging, telemetry and protocol (depends on WP0)

Files: `Core/Src/Managers/StructManager.c`, `Core/Inc/Managers/StructManager.h`, `Core/Src/Tasks/FlashLoggingTask.c`, `Core/Src/Tasks/SDLoggingTask.c`, `Core/Src/Tasks/TelemetryTask.c`, `PROTOCOL.md`.
- D31 and D32: flash records with `Raw`, the snapshot record and when it is written (PRELAUNCH entry), SD records with `Raw` and `Cal`, telemetry with `Cal` and the new `CalStatus` (uint16), packet length and `PROTOCOL.md` updated together.
- Coordinate the snapshot page format with WP-B.

### WP-G: Ground software and protocol consumers (external, can start after WP-F fixes the format)

- New state value 12 (DEEP_CALIBRATION).
- Telemetry `Accel*` and `Gyro*` fields are now the `Cal` values (body frame). Truncation to int16 is intended.
- New `CalStatus` field (uint16): validity flags, HIL pre-seed flag, current tumble pose (0 to 6). Packet length grows by 2 bytes.
- No new commands.
- Flash dump tool: `Raw` accel and gyro in flash records, a snapshot record type, and a calibration sector excluded from the log range.

### WP-H: HIL and bring-up (depends on WP0, then grows with the other packages)

Files: `HIL/hil.py`, `BRINGUP_AND_HIL.md`.
- Scenarios per `BRINGUP_AND_HIL.md`: gesture and tumble, pad calibration, flight profile, negative cases.
- Feeds raw values at the IMU output rate.

## 3. File ownership

| File or area | Owner |
|---|---|
| `shared.h`, `configuration.h` (structure), `StateHandlers.h`, renames across the tree, `tools/build_check.ps1` | WP0 |
| `ImuTumbleCal.*`, CMSIS QR sources | WP-A |
| `W25Q32JV*` | WP-B |
| `Buzzer*.c`, `BuzzerTask.*` | WP-C |
| `13DeepCalibrationStateHandler.c`, `StateManager.c`, `1IdleStateHandler.c` | WP-D |
| `FlightData.c`, `Calibrations.c`, `Calculations.*`, states 2, 3 and aborts, `IIM42653Handler.c` | WP-E |
| `StructManager.*`, logging and telemetry tasks, `PROTOCOL.md` | WP-F |
| `HIL/hil.py`, `BRINGUP_AND_HIL.md` | WP-H |
| `main.c` | WP-C (buzzer task line) and WP-E (timer line): one-line edits, rebase trivially |

The barometric apogee session (`baro-apogee-fix`) owns `6ActiveControlStateHandler.c` and the barometric velocity code. If it needs `FlightData.c`, coordinate with WP-E before merging.

## 4. Interface contracts (proposed)

```c
// shared.h (WP0)
typedef struct {
    float M[9];    // row-major 3x3, CalAccel = M * RawAccel - AccelBiasCal
    float Q[9];    // row-major 3x3 rotation, CalGyro = Q * (RawGyro - GyroBiasRaw)
    bool  Valid;
} ImuCalibration_t;

// ImuTumbleCal.h (WP-A)
void ImuTumble_Reset(ImuTumble_t *T);
ImuTumblePoseStatus_t ImuTumble_CheckPose(const ImuTumble_t *T, uint8_t Pose, const float MeanRaw[3]);
void ImuTumble_AddSample(ImuTumble_t *T, uint8_t Pose, const float RawAccel[3]);
bool ImuTumble_Solve(const ImuTumble_t *T, ImuCalibration_t *Out, ImuTumbleQuality_t *Quality);

// W25Q32JV.h (WP-B)
bool W25Q_CalLoad(float M[9]);          // newest valid record, false if none
bool W25Q_CalAppend(const float M[9]);  // append, erase and rewrite when full
bool W25Q_SnapshotWrite(const FlightSnapshot_t *S);

// Sensors.h (WP-C)
void Buzzer_Play(BuzzerPattern_t Pattern);   // non-blocking
// patterns: BUZZ_DEEPCAL_ENTERED, BUZZ_POSE_1 .. BUZZ_POSE_6, BUZZ_DEEPCAL_OK, BUZZ_DEEPCAL_FAIL

// Calibrations.h (WP-E)
void ImuApplyCalibration(const SystemContext_t *C, const float RawAccel[3], const float RawGyro[3], float CalAccel[3], float CalGyro[3]);
```

Names and signatures may change, but change them in this document and tell the dependent packages.

## 5. Merge order and integration

1. WP0.
2. WP-A, WP-B, WP-C, WP-F, WP-H in parallel.
3. WP-E, then WP-D (D needs the `Cal` flow and the valid flags).
4. Integration on HIL (all scenarios in `BRINGUP_AND_HIL.md`), then hardware bring-up.
5. WP-G ships with the firmware release that changes the telemetry format.

## 6. New macros (created by WP0, one block per package)

`IMU_ODR_HZ`, `DEEP_CAL_HOLD_MS`, `DEEP_CAL_HOLD_AXIS` and sign, `DEEP_CAL_HOLD_G_BAND_PCT`, `DEEP_CAL_HOLD_GYRO_MAX_DPS`, `DEEP_CAL_POSE_SETTLE_MS`, `DEEP_CAL_POSE_SAMPLE_MS`, `DEEP_CAL_POSE_G_BAND_PCT`, `DEEP_CAL_POSE_DOMINANT_MIN_G`, `DEEP_CAL_GYRO_MOTION_MAX_DPS`, `DEEP_CAL_POSE_MAX_RESTARTS`, `DEEP_CAL_TIMEOUT_MS`, `DEEP_CAL_NORM_RESIDUAL_MAX`, buzzer timing macros (`BUZZER_POSE_*`, `BUZZER_DEEPCAL_*`), `STACK_SIZE_BUZZER`, `ACCEL_BIAS_CAL_DISCARD_SAMPLES`, `ACCEL_BIAS_CAL_SAMPLES`, `ACCEL_BIAS_LATERAL_MAX_G`, `CAL_EXPECTED_NOSE_UP_X` (9.81), `W25Q_CAL_SECTOR_ADDRESS` (0x3FF000), `HIL_PRESEED_M` (HIL builds only).

## 7. Definition of done for the whole feature

- All HIL scenarios in `BRINGUP_AND_HIL.md` pass.
- No unprefixed IMU names remain.
- A flash log can be reprocessed offline to reproduce the `Cal` values from `Raw` and the snapshot.
- Hardware bring-up checklist completed once on a real board.
