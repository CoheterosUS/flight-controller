# Bring-up and HIL

Read `DEEP_CALIBRATION_DESIGN.md` for the decisions behind this. This document defines how a board goes from first power-up to PRELAUNCH, what happens when there is no valid calibration, and how HIL exercises the same path.

## 1. Definitions

- Bring-up: first power-up of a board or airframe, or the first power-up after both calibration sectors were erased. The flash has no calibration record, so there is no valid `M`.
- HIL: hardware in the loop. `HIL_MODE 1`. The sensor values are injected over the telemetry UART by `HIL/hil.py` (`COMMAND_HIL_DATA`, GPS through `COMMAND_GPS_DATA`). Serial commands for state changes are not used.
- Flight build: `HIL_MODE 0`. Real sensors.
- Valid calibration: a valid `M` record in the 8 KB flash calibration sector pair, plus valid gyro bias, accel bias and pressure reference measured this boot.

## 2. The rule

PRELAUNCH is rejected unless the calibration is valid. There is no identity fallback in a flight build. With no valid `M`, the CALIBRATION state holds and the telemetry `CalStatus` shows which item is missing. The way out is the deep calibration gesture (nose down and still for 10 s), which always works in pre-flight states.

## 3. What the board does at power-up

| Step | State | What happens | What you observe |
|---|---|---|---|
| 1 | IDLE | Sensors idle, flash initialised, the newest valid `M` is loaded from the calibration sector pair, `SensorsIdleFinished` cleared. Auto-advance when `AUTO_START_CALIBRATION` is on. | Two short beeps (state machine started). A short chirp at each state change. |
| 2 | CALIBRATION | Sensors in performance mode. Pressure reference, gyro bias (raw data) and accel bias (body frame) are measured. Every window restarts if the rocket moves or is not nose up (gross-tilt gate). Holds if there is no valid `M`. | `CalStatus` bits. No audible pattern by default. |
| 3 | PRELAUNCH | Entered only when everything is valid. Kalman initialised (attitude 0, 90, 0), then stepping. | State 2 in telemetry. |
| any of 1 to 3 | DEEP_CALIBRATION | Entered by the 10 s nose-down gesture. See section 4. | One 2 s tone, then the pose prompts. |

## 4. Bring-up procedure (hardware)

Prerequisites: the rocket axes are marked on the fuselage (+Y and +Z). Know where the nose is (+X).

1. Power and sensors. Power up. There must be no flash or sensor fault flags in telemetry (no `W25Q_*`, `IIM42653_*`, `BMP581_*`, `IIS2MDCTR_*` bits). If a flag is set the state goes to GROUND_ABORT: fix the fault first.
2. Raw sanity. Hold the rocket still nose up. `RawAccelY` reads about -9.81 (raw Y points to the tail in the current mounting). Hold it nose down: about +9.81. If the signs are not as expected, stop: the gesture axis (`DEEP_CAL_HOLD_AXIS`) does not match the mounting.
3. Deep calibration. Hold the rocket nose down and still for 10 s. One 2 s tone means entered. Then follow the prompts, one per pose, in this order:

| Prompt (beeps) | Pose | Rocket orientation |
|---|---|---|
| 1 | +X | nose up |
| 2 | -X | nose down |
| 3 | +Y | the +Y mark up |
| 4 | -Y | the +Y mark down |
| 5 | +Z | the +Z mark up |
| 6 | -Z | the +Z mark down |

   After each prompt, orient the rocket and hold it still. Data is ignored during the settle time (default 10 s), then sampled for 30 s. If the prompt repeats, the pose was rejected (moving, or wrong orientation): reorient and hold still. Nominal duration at defaults: about 4 minutes. Success: three 1.2 s tones and a return to IDLE. Failure: one 4 s tone and a return to IDLE. A right-hand-rule mistake in the poses is detected at the end and reported as a failure.
4. Verify the stored calibration. Power cycle. `CalStatus` must show a valid `M` after the IDLE step. Read the record back through the flash dump tool and check the sequence number, the CRC and that `det(Q)` is +1.
5. Pad calibration. Stand the rocket nose straight up and still. CALIBRATION completes: `CalAccelX` is about +9.81 and `CalAccelY`, `CalAccelZ` about 0, `CalGyro` about 0. If it does not complete, check the gross-tilt gate.
6. PRELAUNCH. The state reaches PRELAUNCH. The Kalman filter steps during the wait. Watch the position and velocity outputs for drift over a few minutes (any residual bias integrates twice).
7. Negative checks on the bench: with both calibration sectors erased, the board must stay in CALIBRATION and never reach PRELAUNCH. Nose down and still for 10 s in PRELAUNCH re-enters DEEP_CALIBRATION.

To force a fresh bring-up, erase both calibration sectors (an 8 KB maintenance action; `FLASH_ERASE_ALL` must not erase them, see D18 in the design document).

## 5. HIL

Setup: `HIL_MODE 1`. `hil.py` sends raw accel, gyro, mag, pressure and temperature at the IMU output rate (`IMU_ODR_HZ`). HIL runs the real path: the same state machine, the same solve, the same 8 KB flash calibration sector pair.

The buzzer is not audible in HIL, so `hil.py` drives the tumble by watching the telemetry (state and the pose field inside `CalStatus`) instead of listening.

### 5.1 Scenarios

| ID | Scenario | Pass criteria |
|---|---|---|
| S1 | Gesture and tumble. `hil.py` holds a nose-down still pose for 10 s, then feeds six poses generated through a chosen `M_true` (rotation, scale, skew, offsets, noise). | State goes to DEEP_CALIBRATION (12), the pose field steps 1 to 6, the solved `M` matches `M_true` within a set tolerance, the record is written, the state returns to IDLE, and after a simulated reboot the same `M` is loaded. |
| S2 | Pad calibration. Static nose-up data with a known accel and gyro bias. | `CalAccel` within tolerance of (+9.81, 0, 0), `CalGyro` within 0.01 dps of zero, `CalStatus` all valid, PRELAUNCH reached. |
| S3 | Gyro bias frame regression. A 90 deg rotation about Z in `M_true` with a constant raw gyro offset of (1, 2, 3) dps at rest. | `CalGyro` within 0.01 dps of zero (about 3.2 dps before the fix). |
| S4 | Flight profile. Nominal boost, coast, active control, apogee, main parachute, landed, with sensor noise. | Correct transitions using `CalAccelX`, main parachute at 450 m AGL, Kalman stepping only from PRELAUNCH through MAIN_PARACHUTE. |
| S5 | Abort. Trigger a fault flag during PRELAUNCH. | GROUND_ABORT, pyros safed, the Kalman filter stops propagating. |
| S6 | Negative cases. No `M`; mirrored pose order; motion during sampling; a wrong-way pose; pose timeout; too many restarts; gesture while in BOOST. | No `M`: PRELAUNCH refused. Mirrored order: rejected (`det(Q)` near -1). Motion or wrong pose: the pose restarts with its prompt. Timeout or too many restarts: failure and IDLE. Gesture in BOOST: ignored. |
| S7 | Re-run. Run S1 again with a different `M_true`. | A second record is appended, the newest wins, the older one is kept until the sector is full. |

### 5.2 Pre-seeded calibration (optional, HIL only)

For quick iteration on the flight logic alone, a compile flag `HIL_PRESEED_M` loads an identity `M` into RAM at boot. Rules (assumption to confirm):
- Only when `HIL_MODE 1`. A build with `HIL_PRESEED_M` and `HIL_MODE 0` must fail to compile (`#error`).
- It never writes the flash calibration sectors.
- It sets a flag in `CalStatus` and in the flight snapshot, so any log or telemetry from such a run is identifiable.
- The full scenarios above must still pass without it before a release.

## 6. Flight build checklist

- Set `FLIGHT_BUILD 1` and compile. The compile-time guard proves `HIL_MODE 0`, `HIL_PRESEED_M 0` and `EXTERNAL_COMMANDS 0`.
- Confirm CalStatus bits 6 and 7 are clear in telemetry.
- Confirm the calibration valid bits are set.
- Confirm the state reaches PRELAUNCH.

## 7. Notes

- Serial commands are disabled in the flight configuration. `EXTERNAL_COMMANDS` defaults to 0 in `configuration.h`.
- HIL inputs are raw hardware-axis values. The rotation to the body frame comes from `M`, not from `IMU_ROT_*`.
- `hil.py` and the firmware share the packet layout in `Core/Inc/HIL/HIL.h`. Keep them in step.
