# HIL test plan (kalman-filter branch)

For the HIL agent. This is the single entry point. It covers what must be fixed in the harness first (section 1), the setup (section 2), and every scenario to run (sections 3 to 7). Background: `BRINGUP_AND_HIL.md` (bring-up and S1 to S7), `HIL/APOGEE_HIL.md` and `HIL/APOGEE_SUMMARY.md` (apogee scenarios H1 to H15), `DEEP_CALIBRATION_DESIGN.md` (decisions), `PROTOCOL.md` (wire formats).

Status of the firmware: everything below has passed host tests, the ARM compile check and the full link. Nothing has run on the target yet. Expect real findings, and report each one with the run log.

## 1. Harness status

`HIL/hil.py` was rewritten for this branch (54 byte telemetry with `CalStatus`, states 11 and 12, raw hardware-axis inputs generated from an `M_true` preset, pose-driven tumble, scenario runner with pass or fail and exit codes, CSV logs in `HIL/logs/`). It was self tested only against its built-in mock board (`--port sim`), never against the target. Firmware change for HIL: telemetry runs at 10 Hz when `HIL_MODE 1` (1 Hz otherwise), so events can be timed.

| # | Item | Status |
|---|---|---|
| B1 | 54 byte telemetry, `CalStatus`, pose field | Done in `hil.py`. |
| B2 | State names 11 and 12 | Done. |
| B3 | Raw inputs through `M_true` | Done. Presets `default`, `alt` (S7), `roll90` (S3), `identity`. |
| B4 | Scenario runner, pass or fail, exit codes | Done. |
| B5 | `COMMAND_HIL_BARO` (0x11, 50 Hz) | Done. `COMMAND_HIL_DATA` now carries IMU and magnetometer only (36 byte payload); pressure and temperature go in `COMMAND_HIL_BARO` (8 byte payload) at `--baro-rate` (default 50 Hz). One frame is one barometer sample, so the apogee confirmation time matches the hardware (5 samples = 100 ms). Barometer injections (`--dip-samples`, `--invalid-count`) count barometer samples. H5 stops sending barometer frames (H5b repeats the last value). |
| B6 | Rate | `hil.py` sends at 200 Hz (`--rate`, must equal `IMU_ODR_HZ`). Link load: 200 x 41 B IMU frames plus 50 x 13 B barometer frames, 8850 B/s, 77 % of the 115200 baud link (host pacing measured at 200.0 Hz). Watch for lost packets: the firmware re-arms the receive DMA after each idle event, so bytes that arrive while the telemetry task is preempted are dropped. If a tumble pose keeps restarting with no motion, suspect this (a pose needs 3000 distinct samples in 30 s). |
| B7 | Buzzer not audible | The tumble follows the pose field (CalStatus bits 8 to 10). |

Do not use serial state commands to move the state machine: the flight configuration and the default `EXTERNAL_COMMANDS 0` ignore them. Everything is driven by injected sensor data.

### 1.1 Commands

Common: `python HIL/hil.py --port COM5 --scenario <name> [options]`. Add `--seed N`, `--repeat N`. Exit code 0 pass, 1 fail, 2 setup error (no telemetry, or not a HIL build).

| Test | Command |
|---|---|
| Harness self test | `--port sim --scenario calibrate` (and any other scenario) |
| S1 + S2 | `--scenario calibrate` (tumble, CalAccel check per pose, pad calibration, gyro axis check) |
| S3 | `--scenario calibrate --mounting roll90` (90 deg about the nose, raw gyro offset 1, 2, 3 dps) |
| S2 only | `--scenario pad` |
| S4 | `--scenario flight` (runs the pad first if not in PRELAUNCH) |
| S4b | `--scenario pad_wait --duration 600` |
| S6b | `--scenario tumble --mirrored` (poses 3 and 4 swapped) |
| S6c | `--scenario tumble --motion-pose 3` |
| S6d | `--scenario tumble --wrong-pose 3` |
| S6f | `--scenario tumble --restart-forever 2` |
| S6h + S7 | `--scenario regesture --mounting alt` |
| H1 to H15 | `--scenario flight --hid H1` (presets: H1, H2 (20 seeds), H3, H3up, H4, H4b, H5, H5b, H6, H7, H8, H9, H10, H10b, H12, H13, H14, H15). Change H14 with `--invalid-kind nan|inf|zero|neg|low|high|tnan|t500 --invalid-count N --invalid-at T`. |
| R1 | `--scenario commands`, and in flight `--scenario flight --drogue-cmd-at 1.0` |
| R8 | `--scenario flight --nan-imu-at 12 --nan-imu-count 20` and H14 variants |

Negative tumble cases (S6b to S6f) must start from erased calibration sectors, otherwise an older valid `M` hides the failure (bit 0 stays set). S6a, S6e, S6g, S5 and the flash checks R4 to R7 and R9 need manual steps (sector erase, flash dump, fault provocation); they are not automated yet.

## 2. Setup

- Firmware build: `HIL_MODE 1`, `FLIGHT_BUILD 0`, `EXTERNAL_COMMANDS 0`, `HIL_PRESEED_M 0` (defaults). Do the full run without pre-seed. `HIL_PRESEED_M 1` is allowed only for quick iteration on flight logic and never counts as a pass (CalStatus bit 6 marks it).
- Both calibration sectors (`0x003FE000`, `0x003FF000`) erased at the start of the first run (bring-up state). `FLASH_ERASE_ALL` does not touch them: use the dedicated maintenance action.
- Log everything: the raw telemetry stream to a file (timestamped), the injected truth, the seed, the firmware git hash and the build defines. A failed scenario must be reproducible from the seed.
- Tolerances given below are starting points. If a tolerance is missed by a small factor, report the measured value, do not loosen it silently.

## 3. Deep calibration scenarios (S1 to S3, S6, S7)

Define `M_true` (3x3 with rotation, per-axis scale 0.97 to 1.03, small skew) plus an offset vector `b_true` in m/s2 (up to 0.3 g). Raw accel for a pose with body-frame specific force `f` is `M_true^-1 * (f + b_true)`. Body-frame specific force of an axis pointing up is +9.81 on that axis. Noise: 0.05 m/s2 accel, 0.01 dps gyro, per sample.

| ID | Scenario | Steps | Pass criteria |
|---|---|---|---|
| S1 | Gesture and tumble | Boot with empty calibration sectors. Hold the nose-down still pose (raw AccelY about +9.81, others about 0, gyro about 0) for 10 s, then feed the six poses in the order +X, -X, +Y, -Y, +Z, -Z. React to the pose field: hold still after each prompt for at least the settle time plus 30 s. | State 12 entered after 10 s. Pose field steps 1 to 6. State returns to IDLE (0) and `CalStatus` bit 0 (IMU_CAL_VALID) is set. Solved `M` matches `M_true` within 1 % per element (read it back with the flash dump tool). Power cycle (reset the board, keep the flash): the same `M` loads. |
| S2 | Pad calibration | After S1, feed static nose-up data with a known gyro bias (for example 0.3, -0.2, 0.1 dps) and an accel offset. | Reaches PRELAUNCH (2). `CalStatus` bits 0 to 5 set. `CalAccelX` about +9.81, `CalAccelY` and `CalAccelZ` about 0 (within 0.1), `CalGyro` within 0.01 dps of zero (telemetry truncates to int16, so check the flash log or the SD log for the fine values). |
| S3 | Gyro bias frame regression | `M_true` includes a 90 degree rotation about Z, with a constant raw gyro offset of (1, 2, 3) dps at rest. | `CalGyro` within 0.01 dps of zero (about 3.2 dps if the bias is applied in the wrong frame). |
| S6 | Negative cases | Run each separately, from a clean state. (a) Empty calibration sectors, static nose-up data: PRELAUNCH must never be reached, board holds in CALIBRATION (1) with bit 0 clear. (b) Mirrored pose order (right-hand rule violated): tumble fails, IDLE, no record written (`det(Q)` near -1). (c) Motion during sampling (add 20 dps gyro or an accel disturbance for 3 s): the pose restarts with its prompt, then succeeds after a still period. (d) Wrong-way pose (feed -Y when +Y is expected): the pose is rejected and restarts. (e) Pose timeout: stop feeding valid data: failure and IDLE. (f) Too many restarts (more than `DEEP_CAL_POSE_MAX_RESTARTS`): failure and IDLE. (g) Nose-down still for 10 s while in BOOST: ignored, state unchanged. (h) The same gesture in PRELAUNCH: re-enters DEEP_CALIBRATION. | Each case as stated, with no fault flags set by the case itself. |
| S7 | Re-run | Run S1 again with a different `M_true`. Then repeat the tumble 5 more times. | A new record each time, the newest always wins after a reset, the older records remain until the sector rolls over. Check the sequence number increases by one per calibration. |

Extra checks folded into S1 and S7 (audit fixes):
- NaN and infinity: inject a single NaN accel sample and a single +inf gyro sample during a pose and during the pad calibration. The window restarts, no bias becomes valid from it, no fault, no NaN in telemetry.
- Constant rotation: feed a steady 20 dps gyro at rest. The gyro bias is not accepted (bit 1 stays clear until the rotation stops).
- Nose down or upside down during pad calibration: accel bias not accepted (bit 2 clear), so no false boost later.
- A calibration sector rollover: after 64 successful calibrations (a fast loop in a scripted session is fine, use pose time overrides only if the build has them, otherwise run overnight or arrange the count by direct flash pre-fill with the flash tool), the next record goes to the other sector and the newest still loads. Cut power (reset the board) during the rollover erase and confirm an older valid record still loads.

## 4. Flight scenarios (S4, S5)

After a good S1 and S2 (calibrated, PRELAUNCH).

| ID | Scenario | Pass criteria |
|---|---|---|
| S4 | Nominal flight profile | Sequence PRELAUNCH, BOOST, COAST, (ACTIVE_CONTROL if above its gate), APOGEE, MAIN_PARACHUTE, LANDED. Boost detection uses `CalAccelX` above +20 m/s2 (`PRELAUNCH_BOOST_ACCEL_X_THRESHOLD`) and coast is below +5 (`BOOST_COAST_ACCEL_X_THRESHOLD`). Main parachute at 450 m AGL. Kalman stepping bit (5) set from PRELAUNCH to MAIN_PARACHUTE, cleared in LANDED. Position and velocity from the filter stay sane (no NaN, no divergence) for the whole profile. |
| S4b | Pad wait | Stay in PRELAUNCH for 10 minutes with static data. Kalman velocity and position drift stays bounded (record the drift, a small drift is expected from residual bias). No spurious BOOST. |
| S5 | Abort | Trigger a fault during PRELAUNCH (for example inject an invalid baro reference or force a flash fault via the test hook if one exists, or stop the IMU stream if a stale-sensor fault exists). Expect GROUND_ABORT (9), pyros safed, Kalman stepping bit cleared. Repeat for an abort during ascent (state 11). |

## 5. Apogee scenarios (H1 to H15)

Run exactly as written in `HIL/APOGEE_HIL.md`. Order: H1 and H11, then H3, H4, H14, then H5, H6, then H7, H8, then H2, H12, H13, H15, then H9 and H10. Use the atmosphere model from `APOGEE_HIL.md` section 2 for the pressure.

## 6. Audit regression checks (new, from the pre-flight audit)

| ID | Check | Pass criteria |
|---|---|---|
| R1 | Serial commands are refused. Send `CA FE 04 00 BE` (drogue), reset, ground-abort and calibration frames in every state on the pad and during flight. | The state does not change and no pyro fires. With `EXTERNAL_COMMANDS 0` (default) all of them are ignored. If a build with `EXTERNAL_COMMANDS 1` is tested, drogue and landed work only when `HIL_MODE 1`. |
| R2 | Build guards. Compile with `-DFLIGHT_BUILD=1` on the default config. | The compile fails with the `FLIGHT_BUILD requires ...` error. (Also verified by `python tools/link_check.py . -DFLIGHT_BUILD=1`.) A build with `FLIGHT_BUILD 1 HIL_MODE 0 EXTERNAL_COMMANDS 0 HIL_PRESEED_M 0` links. |
| R3 | CalStatus bits 6 and 7. | Bit 7 set in every HIL build telemetry, bit 6 only with pre-seed. In a flight build both clear (bench check, not HIL). |
| R4 | Flash flush on stop. Run a full flight to LANDED, then stop. | The flash log contains the last records (the partial last page is written). The SD or flash dump shows the record around landing. |
| R5 | Log full. Fill the log region (fast fill with the flash tool) and boot. | `W25Q_LOG_FULL` (fault bit 12) is raised, the board does not arm (no PRELAUNCH or it aborts). |
| R6 | Flight snapshot. After entering PRELAUNCH with valid calibration. | The snapshot page is written and verifies (no bit 14 fault). Flash log has the raw fields plus a once-per-flight snapshot with `M`, biases and the pressure reference. |
| R7 | No empty flights. Return to IDLE repeatedly (several tumble calibrations, several resets). | The number of flight markers does not grow with each return to IDLE when no data was logged in between. A real flight still gets one marker. |
| R8 | Calibration NaN safety in flight. Inject NaN or inf into accel, gyro and pressure for 1 to 20 samples during COAST. | State machine keeps running, no false apogee or main chute, telemetry values stay finite (the last good value is held), Kalman does not step on non-finite input. |
| R9 | Fault flags under load. Provoke two faults from different tasks at the same time (for example a flash write failure while a sensor fault arrives). | Both bits appear in `Flags` (atomic OR). |
| R10 | Boot path when a calibration exists. | IDLE loads `M`, `SensorsIdleFinished` behaves (state does not hang in IDLE, CALIBRATION runs), two short beeps, state chirps do not cut the 2 s entry tone (observe on the bench with the buzzer if possible). |

## 7. Reporting

Per scenario: ID, seed, firmware hash, build defines, pass or fail, measured values against the criteria, the telemetry log file name, and for a failure the first telemetry sample where behaviour diverged. Put the results in `HIL/RESULTS_<date>.md` on a branch named `hil-results`, not on `kalman-filter`. Do not change firmware behaviour to make a scenario pass: report it. Any change to thresholds goes through the firmware owners.

Run order for the first session: the harness self test (`--port sim`), then S1, S2, S3 (calibration on the target), then S4, S5, then R1 to R3, then S6, S7, then the R checks, then the apogee scenarios.
