# Deep calibration and calibrated data flow: design

Status: design settled 2026-09-27, implementation not started. Branch: `kalman-filter`.

Companion documents:
- `DEEP_CALIBRATION_WORK_PACKAGES.md`: how to build it concurrently (work packages, file ownership, interfaces).
- `BRINGUP_AND_HIL.md`: first power-up, what happens with no calibration, and the HIL scenarios.
- `BACKLOG.md`: open defects (barometric apogee detection, tracked on branch `baro-apogee-fix`).

Source method: ST AN4508, "Parameters and calibration of a low-g 3-axis accelerometer" (six stationary positions, least squares, 12 parameters).

Status legend used below:
- SETTLED: decided by the team.
- DEFAULT: proposed value or detail, not contradicted, adjustable through `configuration.h`. Confirm when implemented.
- OPEN: needs a decision.

## 1. What this adds

1. A new state, `STATE_DEEP_CALIBRATION` (value 12), entered by holding the rocket nose down and still for 10 s. It runs a six-pose tumble test on the IMU, computes a 3x3 accelerometer map `M`, stores it in flash and returns to IDLE. It is run once before launch, and can be re-run at any time on the ground.
2. A calibrated data flow: every IMU quantity exists in two forms, `Raw` (hardware axes, exactly as the IMU reports it) and `Cal` (rocket body axes, corrected). Flight logic and the Kalman filter use `Cal`. Logs keep `Raw`.
3. Per-boot bias calibration in the CALIBRATION state: gyro bias on raw data, accelerometer bias in the body frame with the nose straight up.
4. Kalman gating: the filter propagates from PRELAUNCH on, and stops in abort states.
5. A non-blocking buzzer task with the tumble prompts.
6. Fixes found on the way: gyro bias frame bug, `SensorsIdleFinished` never cleared, IMU timer and Kalman step not derived from the IMU output rate, main parachute threshold.

## 2. Frames and conventions (SETTLED)

- Raw frame: the IIM42653 hardware axes. In the current mounting raw +Y points toward the tail, so nose up reads -9.81 m/s2 on raw Y and nose down reads +9.81 m/s2.
- Cal (body) frame: rocket axes, forward-right-down. +X is the direction the rocket points (nose), +Y and +Z are marked on the fuselage, right hand rule in case of doubt. This matches the Kalman initial attitude (pitch 90 deg means body X up).
- Acceleration convention: the Kalman filter convention, specific force. An axis pointing up reads +g. Nose up reads +9.81 on `CalAccelX`. `g` is the Kalman constant 9.81, not the local value.
- Pose names: pose "+X" means rocket +X pointing up (toward the sky), so the ideal reading is +9.81 on X. This is the sign flip relative to AN4508 Table 1, which uses "axis pointing down reads +1 g".

## 3. Decisions

### 3.1 Tumble method

| ID | Decision | Status |
|---|---|---|
| D1 | AN4508 model: `[Ax1 Ay1 Az1] = [Ax Ay Az 1] * X`, X is 4x3. Solved by least squares `X = (w^T w)^-1 w^T Y`. Accumulate per-pose sums only (count, sum of raw, sum of outer products), solve the 4x4 in double at the end. | SETTLED |
| D2 | Six poses, in this order: +X, -X, +Y, -Y, +Z, -Z, in rocket axes, using the fuselage marks. 30 s of data per pose. | SETTLED |
| D3 | IMU data is ignored for an adjustable settle time after each prompt beep. Default 10 s (`DEEP_CAL_POSE_SETTLE_MS`). | SETTLED, value DEFAULT |
| D4 | Pose validation, independent of the (unknown on first run) IMU-to-rocket mapping. After the settle time the still-window mean must have: magnitude within a band of 9.81 (default 10 percent); one axis carrying at least 0.8 g; consistency with previous poses (pose 2 opposite to pose 1, poses 3 and 4 perpendicular to X and opposite each other, poses 5 and 6 on the third axis and opposite each other). Otherwise the pose timer restarts and the prompt is replayed. | SETTLED, values DEFAULT |
| D5 | Motion check: if the raw gyro norm exceeds a limit during sampling, the pose sums are discarded and the pose restarts with its prompt. Default limit 2 dps. A maximum number of restarts per pose (default 5) and an overall timeout (default twice the nominal duration) end in failure. | SETTLED, values DEFAULT |
| D6 | `M = X3^T` (the 3x3 part of X, transposed to column-vector form). A QR decomposition of `M` (CMSIS-DSP `arm_mat_qr_f32`) gives `Q`, the rotation used for the gyro. The signs are normalized so that the diagonal of `R` is positive and `det(Q) = +1`. A result with `det(Q)` not near +1 (a mirrored pose sequence), a per-pose residual norm error above a limit, or a badly conditioned solve is rejected. | SETTLED |
| D7 | The tumble's offsets (4th row of X) are discarded. The pad measures the biases fresh. | SETTLED |

### 3.2 Trigger, state and exits

| ID | Decision | Status |
|---|---|---|
| D8 | Trigger is a gesture only: rocket nose down (raw AccelY within a band around +9.81 m/s2, other axes small), still (gyro below a limit), for 10 s. Serial commands are disabled in the flight configuration, so there is no deep calibration command. | SETTLED |
| D9 | The gesture always applies, including when a valid `M` exists, so a re-run is always possible. It is accepted only in pre-flight states (IDLE, CALIBRATION, PRELAUNCH), never from BOOST onward. The hold is timed with tick counts, because `ConfirmCounter_t` is 8 bits (about 1 s). | SETTLED |
| D10 | Exits always go to IDLE (success, failure, timeout, too many restarts). A non-zero `SystemFaultFlags` goes to GROUND_ABORT as in the other pre-flight states. Pyros are safed on entry. | SETTLED |
| D11 | On entry: Kalman stepping stops and the Kalman is marked not initialised, flash flight logging is disabled, sensors are put in performance mode. | SETTLED |
| D12 | The state value is appended after `STATE_ASCENT_ABORT` so existing values do not change: DEEP_CALIBRATION = 12. Handler file `13DeepCalibrationStateHandler.c`. | SETTLED |

### 3.3 Buzzer

| ID | Decision | Status |
|---|---|---|
| D13 | The prompt for pose k is k beeps (k = 1 to 6). Default 400 ms on, 400 ms off. | SETTLED, timing DEFAULT |
| D14 | Other patterns, chosen to be unmistakable against 1 to 6 short beeps: entered = one 2 s tone; success = three 1.2 s tones; failure = one 4 s tone. All longer than 400 ms. The earlier proposal (4, 6 and 8 beeps) collided with the pose counts. | DEFAULT |
| D15 | Patterns play from a dedicated low-priority buzzer task, sent by task notification. `Buzzer_Beep_Counter` blocks its caller and toggles the pin, so nothing in the 250 Hz loop may call it for these patterns. | SETTLED |

Existing patterns stay as they are: state change chirp 5 ms, boot 2 beeps, flash maintenance 2, 3 or 5 beeps.

### 3.4 Storage

| ID | Decision | Status |
|---|---|---|
| D16 | Only `M` is stored (9 floats). No gyro bias, no accel bias, no offsets. `Q` is derived at load by the QR. | SETTLED |
| D17 | Two adjacent 4 KB sectors at the end of the flash (`0x3FE000` and `0x3FF000`), 8 KB total, used as an append-only ping-pong pair. Records are 64 bytes: magic, version, sequence number, `M[9]` and CRC32. The newest record with a valid CRC wins. When the active sector is full, the other sector is erased and receives the next record before the old sector can be touched, so rollover cannot erase the last valid calibration. | SETTLED |
| D18 | Consequences for the log code: `W25Q_LOG_END` is `0x3FE000` (total size minus 8192) and bounds `W25Q_HasSpace` and the write-pointer scan. `W25Q_EraseAll` must preserve both calibration sectors. `W25Q_DumpToSD` must skip both sectors. | SETTLED |

### 3.5 Per-boot calibration (CALIBRATION state)

| ID | Decision | Status |
|---|---|---|
| D19 | Gyro bias is measured on raw data (`RawGyro`) and subtracted in the raw frame, then `Q` is applied: `CalGyro = Q * (RawGyro - GyroBiasRaw)`. This fixes the existing bug where the bias was measured after `ApplyIMURotation` and subtracted before it. | SETTLED |
| D20 | Accel bias is measured in the body frame after `M`, with the nose straight up, on all three axes: `AccelBiasCal = mean(M * RawAccel) - (+9.81, 0, 0)`. Applied as `CalAccel = M * RawAccel - AccelBiasCal`. Same discard and sample counts as the existing calibrations. The lateral tilt false bias is an accepted compromise (1 deg of tilt is 0.17 m/s2). | SETTLED |
| D21 | A gross-tilt sanity gate: if a lateral component exceeds a limit (default 0.1 g, about 6 deg) or the rocket is not still, the window restarts. CALIBRATION waits until the rocket is placed nose up and still. | SETTLED, value DEFAULT |
| D22 | PRELAUNCH is rejected unless `M`, the gyro bias, the accel bias and the pressure reference are all valid (and the GPS fix, if `GPS_FIX_REQUIRED`). There is no identity fallback. See `BRINGUP_AND_HIL.md`. | SETTLED |
| D23 | `IMU_ROT_*` and `ApplyIMURotation` are retired once `M` exists. `M` and `Q` include the mounting rotation. | SETTLED |

### 3.6 Kalman filter

| ID | Decision | Status |
|---|---|---|
| D24 | Initial attitude stays hardcoded (`KALMAN_INITIAL_ROLL/PITCH/YAW_DEG` = 0, 90, 0). It is not levelled from gravity. | SETTLED |
| D25 | Initialised at the end of CALIBRATION, when all calibrations are valid. `kalman_filter()` steps from PRELAUNCH through MAIN_PARACHUTE, including the wait on the pad. It stops in IDLE, CALIBRATION, DEEP_CALIBRATION and all abort states. Initialised and stepping are two separate conditions. | SETTLED |
| D26 | LANDED: stop stepping (the sensors are turned off in that state anyway). | DEFAULT |
| D27 | The filter step is `1 / IMU_ODR_HZ`, not `LOOP_DT`. Today `KALMAN_DT` is 4 ms while the IMU produces a new sample every 5 ms (ODR 200 Hz), so each update integrates 20 percent too little time. | DEFAULT (found by code reading, confirm on hardware) |

### 3.7 Data flow, naming, logging

| ID | Decision | Status |
|---|---|---|
| D28 | Names: prefix `Raw` means hardware axes, straight from the driver, physical units, nothing applied. Prefix `Cal` means body frame after `M`/`Q` and biases. No unprefixed IMU names remain (`RawAccelX`, `CalAccelX`, `RawGyroX`, `CalGyroX`, `RawMagX`, `GyroBiasRawX`, `AccelBiasCalX`). Renaming makes the compiler flag every use to re-point. | SETTLED |
| D29 | `Cal` fields are computed every loop, independent of the Kalman filter. The tumble and the gyro bias calibration read the `Raw` fields. | SETTLED |
| D30 | Flight thresholds move to `CalAccelX`, with the sign flipped. Boost: `CalAccelX > +20 m/s2` (was raw Y below -20). Burnout: `CalAccelX < +5 m/s2` (was raw Y above -5). Counts and values otherwise unchanged. This resolves the README item "check transition conditions against IMU reference frame". | SETTLED |
| D31 | Flash log records carry `Raw` accel and gyro (floats, record stays 56 bytes). At PRELAUNCH entry, one snapshot record is written into the log with the values used that flight: `M`, `AccelBiasCal`, `GyroBiasRaw`, reference pressure and temperature, ODR, config version. Offline, `Cal = f(Raw, snapshot)`. | SETTLED, layout DEFAULT |
| D32 | SD log records carry both `Raw` and `Cal`. Telemetry sends the `Cal` fields (truncation to int16 is intended) plus a new `CalStatus` (uint16: validity flags, HIL pre-seed flag, HIL mode flag, current tumble pose). | SETTLED, CalStatus layout DEFAULT |

### 3.8 Other changes

| ID | Decision | Status |
|---|---|---|
| D33 | Main parachute deployment altitude set to 450 m AGL (`APOGEE_MAIN_PARACHUTE_BAROM_ALT_THRESHOLD`). Done in `configuration.h`, uncommitted. The disabled GPS threshold (450 ASL, baseline 90 m) was not changed and does not match AGL. | DONE |
| D34 | IMU timer period derived from `IMU_ODR_HZ` (one macro selects both the ODR register value and the timer period). Today the timer is 4 ms and the ODR is 200 Hz. | SETTLED |
| D35 | `SensorsIdleFinished`: cleared in `OnStateEntry` before the notification when entering IDLE, and set by `HandleSensors` in both HIL and hardware builds after the idle step when `AUTO_START_CALIBRATION` is on. Today it is never cleared, and in HIL builds it is never set. | DEFAULT |
| D36 | Barometric apogee detection is unacceptable as is. Fixed on a separate branch (`baro-apogee-fix`) in a separate session. See `BACKLOG.md`. | SETTLED |

## 4. Accepted limitations

- Hand-held poses: tilt in a pose enters the fit as cross-axis error (AN4508 assumes exact positions).
- Pad tilt is not separated from lateral accel bias (D20).
- Temperature drift between the tumble and the pad is only covered by the per-boot bias.
- If the IMU is remounted, `M` is no longer valid and the tumble must be redone.
- A gesture false trigger needs 10 s of nose-down stillness. Carrying the rocket is not still.

## 5. Open items

- Numeric defaults (thresholds, settle time, restart limits): to be tuned on hardware.
- D14, D26, D27, D31 layout, D32 `CalStatus` layout, D35: defaults awaiting confirmation.
- Source and version of the CMSIS-DSP files `arm_mat_qr_f32.c` and its Householder helper (the prototypes already exist in `matrix_functions.h`).
- Whether SD logging stays disabled (`SD_LOGGING_ENABLED 0`).
