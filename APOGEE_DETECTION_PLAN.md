# Apogee detection plan

Branch: `baro-apogee-fix` (local, not pushed). Tracks the BACKLOG item "Fix barometric apogee detection". Status: plan agreed, implementation delegated to Codex in three units (section 7). Contract header: `Core/Inc/Utils/ApogeeDetector.h`.

## 1. Problem (found by code reading, not reproduced on hardware)

- The only enabled apogee trigger is `BarometricVelocity <= 0` in `6ActiveControlStateHandler.c`, one sample, no confirmation.
- `BarometricVelocity` is recomputed every loop iteration (4 ms) but the BMP581 publishes every 20 ms, so 4 of 5 iterations give exactly 0. A false apogee (and a drogue fire) is almost immediate on entering `ACTIVE_CONTROL`.
- Apogee can only be reached from `ACTIVE_CONTROL`, and `ACTIVE_CONTROL` is only reached from `COAST` above 2000 m. If the rocket never gets there, or the IMU never lets `BOOST` end, nothing ever fires the drogue.

## 2. Decisions

| ID | Decision |
|---|---|
| A1 | Two independent channels only: B (barometric drop from peak) and D (timer). GPS and the Kalman filter are untested and are NOT used. |
| A2 | Rule: drogue fires when B or D is true. No voting, nothing else can trigger `APOGEE` (apart from the existing manual `COMMAND_DROGUE`). |
| A3 | No minimum altitude inhibit. B only works relative to the running peak, so it does not depend on reaching any altitude. |
| A4 | `ACTIVE_CONTROL` is an EuRoC feature and must never gate apogee. The detector is state independent, see section 3. |
| A5 | Drogue firing itself is verified and out of scope. |
| A6 | Verification is by HIL profiles, defined in `HIL/APOGEE_HIL.md`. |
| A7 | Altitudes tied to the competition are placeholders until the EuRoC rules are released, see section 6. |
| A8 | `APOGEE_TIMER_MS = 28000`, measured from BOOST entry. It must exceed the latest plausible apogee time, otherwise D fires a drogue while the rocket is still climbing. Confirm against the flight simulation. |
| A9 | The trigger channel is logged only (SD log, flash log if there is room), NOT in telemetry. Values: none, baro, timer, command. |
| A10 | The barometer mailbox gets a sample counter. Every consumer works on new samples only. |
| A11 | The landing check is fixed: barometric velocity is computed on new valid samples only, over a window, and the landed confirmation counts new valid samples. |
| A12 | Baro rate in HIL is 50 Hz, on its own command, like the real sensor. |
| A13 | Invalid pressure never becomes an altitude of 0 m. See section 4.1. |

## 3. Where the transition lives

Current chain: `BOOST` -> `COAST` (burnout, IMU) -> `ACTIVE_CONTROL` (2000 m) -> `APOGEE`.

Proposal: a single `ApogeeDetector` is called from `HandleState` in `StateManager.c` before the per-state dispatch, whenever the state is `BOOST`, `COAST` or `ACTIVE_CONTROL`. It can return `STATE_APOGEE` from any of them. The per-state handlers keep only their own transitions (`ACTIVE_CONTROL` no longer contains any apogee logic).

| State | Peak tracking | Channel B | Channel D |
|---|---|---|---|
| `BOOST` | yes | no (motor burning, pressure artefacts possible, altitude cannot be falling) | yes |
| `COAST` | yes | yes | yes |
| `ACTIVE_CONTROL` | yes | yes | yes |

Why this placement:
- `COAST` without `ACTIVE_CONTROL` (rocket does not reach the gate altitude, or the feature is disabled by the rules) still detects apogee through B.
- D also runs in `BOOST`, so a stuck IMU (burnout never detected, the rocket stays in `BOOST` forever) still gets a drogue. B stays off in `BOOST` on purpose: the burn is short and D covers a stuck `BOOST`.
- The peak is tracked from launch, not from `ACTIVE_CONTROL` entry, so state changes never reset it.
- Not from `PRELAUNCH` or earlier: a false trigger on the pad or a pre-launch fault must not fire a drogue. `APOGEE` is not reachable from the abort states either.
- The detector is reset on `BOOST` entry (peak = current altitude, counters = 0, `t0` = entry tick).

## 4. Detector specification

Pure C module (`Core/Src/Utils/ApogeeDetector.c`, header in `Core/Inc/Utils/`), no HAL, no RTOS, host testable.

```
ApogeeDetector_Reset(alt)
ApogeeDetector_Update(alt, new_baro_sample, baro_allowed, elapsed_since_launch_ms) -> NONE | BARO | TIMER
```

Channel B:
- Only acts when `new_baro_sample` is true (once per barometer publish, not per loop iteration).
- `peak = max(peak, alt)`. If `alt < peak - APOGEE_DROP_M` the counter increments, otherwise it resets. Returns `BARO` when the counter reaches `APOGEE_CONFIRM_SAMPLES`. Only evaluated when `baro_allowed` (state is not `BOOST`).
- Non-finite `alt` is ignored (counter unchanged), never treated as a drop.
- Filter (decided): a median of the last 3 accepted samples, nothing else. It removes isolated outliers without smoothing lag (one sample, 20 ms). No low-pass, no derivative. The BMP581 IIR and oversampling configuration is NOT touched: the raw data is shared with the Kalman filter. The drop margin and the confirmation absorb the remaining noise. The unfiltered altitude noise is about 0.2 to 0.3 m, so the default margin has a large safety factor.

Channel D:
- Returns `TIMER` when `elapsed_since_launch_ms >= APOGEE_TIMER_MS`. Independent of every sensor. `elapsed` is measured from `BOOST` entry (`StateEntryTicks[STATE_BOOST]`).

Expected latency of B (ballistic, near apogee): a 15 m drop takes about 1.75 s (`0.5 * g * t^2`), plus 5 samples (100 ms) and 1 median sample (20 ms), so the drogue fires about 1.9 s after true apogee, at about 17 m/s descent and about 17 m below the peak. With apogee at 25 s in HIL, B fires at about 26.9 s, only about 1.1 s before the 28 s timer. Tight: the timer must stay a backstop.

### 4.1 Invalid barometer data (must never cause a drogue or main chute)

Today `CalculateAltitude` returns 0.0 for invalid pressure or invalid reference. A 0 m altitude looks like a giant drop from the peak (false drogue), and it also satisfies `BarometricAltitude <= 450` in the APOGEE state (false main chute, a single-sample check today). "Invalid" is now defined, per sample, by `BaroSampleValid()`:

1. Pressure is NaN, infinite or <= 0.
2. Pressure is outside `[BARO_VALID_MIN_PA, BARO_VALID_MAX_PA]` (30 to 125 kPa, the BMP581 operating range).
3. Temperature is NaN, infinite or outside `[-40, 85] C`.
4. The pad reference pressure is not valid (`ReferencePressurePaValid` false, <= 0, non-finite).
5. Nothing published yet (sample id 0), or the driver read failed (a failed read must not publish).
6. The derived altitude is non-finite.

On top of that the detector has a slew gate (`APOGEE_BARO_MAX_SPEED_MPS * dt + APOGEE_BARO_SLEW_MARGIN_M` against the last accepted altitude): a value that changes faster than physically possible is rejected the same way, so a bad spike cannot poison the peak (which would make the real altitude look like a drop) or count as a drop.

Consequences:
- An invalid or rejected sample is ignored, never counted as a drop, a confirmation or a landing sample.
- `FlightData.BarometricAltitude` holds the last valid altitude (0 only before the first valid sample), and `FlightData` gets a `BaroValid` flag and `BaroSampleId`.
- Every barometer threshold consumer needs a valid new sample: COAST to ACTIVE_CONTROL (already confirmed, add valid), APOGEE to MAIN_PARACHUTE (add a confirmation of `APOGEE_CONFIRM_SAMPLES` valid samples, it has none today), MAIN_PARACHUTE to LANDED.
- Long invalid data leaves only D. That is intended.
- Not in scope: the Kalman filter also consumes `PressurePa` directly. It is not used for any trigger (A1). Noted in the backlog.

New sample detection: the BMP581 mailbox (`BMP581Mailbox.c`) has no sequence number today. Decided: the mailbox slot carries a 32-bit `SampleId`, incremented on every publish and every HIL inject, written into the slot together with the data (so the id and the data are always consistent). `FlightData.BaroSampleId` exposes it. A new sample is `id != last id`.

## 5. Configuration (`configuration.h`)

| Macro | Default | Note |
|---|---|---|
| `APOGEE_DROP_M` | 15.0 | drop below the filtered peak (was 7.0, changed by the user) |
| `APOGEE_CONFIRM_SAMPLES` | 5 | new barometer samples, about 100 ms at 50 Hz |
| `APOGEE_TIMER_MS` | 28000 | from BOOST entry. Decided by the user. WARNING: the old HIL profile apogee is at about 27.5 s, so D would beat B. The HIL default profile must be rescaled so the nominal apogee is well before 28 s (25 s), see `HIL/APOGEE_HIL.md`. |
| `BARO_ODR_HZ` | 50 | real barometer rate. Confirm the configured BMP581 ODR in performance mode matches |
| `APOGEE_BARO_MAX_SPEED_MPS`, `APOGEE_BARO_SLEW_MARGIN_M` | 400, 10 | slew gate |
| `BARO_VALID_MIN_PA`, `BARO_VALID_MAX_PA`, `BARO_VALID_MIN_TEMP_C`, `BARO_VALID_MAX_TEMP_C` | 30000, 125000, -40, 85 | sample plausibility |

Remove: `ACTIVE_CONTROL_APOGEE_BAROM_VEL_*` as a trigger. The disabled altitude, GPS and delay macros for `ACTIVE_CONTROL` are removed with the handler block to avoid dead options (the delay is replaced by `APOGEE_TIMER_MS`).

## 6. EuRoC dependency (do not forget)

`ACTIVE_CONTROL` is a EuRoC feature and the altitudes in `configuration.h` are placeholders until the judges release the rules:
- `COAST_ACTIVE_CONTROL_BAROM_ALT_THRESHOLD` (2000 m) and the disabled 2900 m thresholds (`ACTIVE_CONTROL_APOGEE_*`, target apogee).
- `APOGEE_MAIN_PARACHUTE_BAROM_ALT_THRESHOLD` (450 m AGL).
- `APOGEE_TIMER_MS` and the HIL profile apogee (`PROFILE_*` in `hil.py`) must be re-derived from the target apogee.

When the rules are released, re-check all of these and re-run the HIL profiles. Also tracked in `BACKLOG.md`.

## 7. Work (Codex, three parallel units, one worktree each)

The interface is fixed by `Core/Inc/Utils/ApogeeDetector.h` and the macros already in `configuration.h`. Briefs are in `C:\Users\carma\codex-runs\prompts\apogee-*.md`.

| Unit | Branch | Owns | Content |
|---|---|---|---|
| core | `apogee-core` | `Core/Src/Utils/ApogeeDetector.c`, `tests/host/test_apogee.c`, `tests/host/run_apogee_tests.ps1` | Detector and `BaroSampleValid` implementation, host tests. |
| fw | `apogee-fw` | BMP581 mailbox and data struct, `FlightData.c/.h` (barometer part only), `shared.h` FlightData and SystemContext fields, `Calculations.c/.h` (baro velocity), `StateManager.c`, state handlers 4 to 8, `StructManager`, SD and flash log record, `configuration.h` (remove old macros) | Sample counter, validity, velocity and landing fix, main chute confirmation, wiring, removal of the ACTIVE_CONTROL apogee logic, channel log field. |
| hil | `apogee-hil` | `HIL/hil.py`, `Core/Src/HIL/HIL.c`, HIL command routing in `Core/Src/Tasks/TelemetryTask.c`, `Core/Inc/Protocol/Protocol.h`, `PROTOCOL.md` | Separate 50 Hz barometer command, scenarios H1 to H15. |

Rules: no unit edits another unit's files. WP-E (calibrated data flow, `FlightData.c`) is not merged yet, so `fw` keeps its `FlightData.c` edits small and isolated in the barometer block. Merged by hand into `baro-apogee-fix`, host tests and `tools/build_check.ps1` run once per batch by the reviewer.

Also in the backlog: GPS velocity has the same per-loop derivative (unused, disabled), Kalman consumes unvalidated `PressurePa`.

## 8. Open items

- Confirm 28 s is later than the plausible apogee time in the flight simulation (D must never fire during the climb).
- Actual BMP581 ODR in performance mode (the `fw` brief asks to verify and report).
- Whether the flash log record has room for the channel byte (else SD log only).
