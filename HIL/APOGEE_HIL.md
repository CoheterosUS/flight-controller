# HIL profiles for apogee detection

For the HIL worker. Read `APOGEE_DETECTION_PLAN.md` first (detector design, channels B and D, where it runs). This file defines the scenarios `hil.py` must be able to run and what each one must show. The firmware must be a `HIL_MODE 1` build. No serial commands are used to change state.

## 1. What is being tested

The firmware fires the drogue (enters `STATE_APOGEE`, 6) when either:
- **B**: barometric altitude stays below `peak - APOGEE_DROP_M` for `APOGEE_CONFIRM_SAMPLES` new barometer samples. Only in `COAST` and `ACTIVE_CONTROL`.
- **D**: time since `BOOST` entry reaches `APOGEE_TIMER_MS`. In `BOOST`, `COAST` and `ACTIVE_CONTROL`.

The detector runs whatever the state (`BOOST`, `COAST`, `ACTIVE_CONTROL`). `ACTIVE_CONTROL` must not be needed. Default parameters: drop 7 m, 5 samples at 50 Hz, timer 28 s (`configuration.h`).

## 2. Changes needed in `hil.py`

- `--scenario <name>` selecting from the table below, `--seed`, and `--apogee-alt` (target apogee), so the profile can be rescaled when the EuRoC rules change the target altitude.
- Ground truth output per run: the true apogee time and altitude from the profile, and the time and altitude when telemetry first reports state 6.
- Pass or fail print per scenario, plus a non-zero exit code on failure.
- Add the missing state names to `STATE_NAMES`: 11 `ASCENT_ABORT` and 12 `DEEP_CALIBRATION` (check `shared.h` for the current enum).
- Baro rate: the barometer is sent at 50 Hz on its own command (`COMMAND_HIL_BARO`, pressure and temperature), like the GPS. `COMMAND_HIL_DATA` keeps IMU and magnetometer at 100 Hz and no longer carries pressure or temperature. Update `PROTOCOL.md`. Each barometer packet is one sample (the firmware assigns the mailbox sample id), so the confirmation time in HIL equals the real one (5 samples = 100 ms).
- Default profile timing: the timer channel is 28 s from BOOST entry, so the nominal profile must reach apogee well before that (target about 20 s from launch, detection about 1.3 s later). Add `--apogee-time` (or derive thrust and burn for it) and keep `--apogee-alt`. The old default profile (about 27.5 s to apogee) would let the timer win and hide B.

## 3. Scenarios

Profile numbers refer to the rescaled default profile (about 3000 m apogee, about 20 s from launch to apogee). All scenarios use noise unless stated.

| ID | Scenario | Injection | Pass criteria |
|---|---|---|---|
| H1 | Nominal | The existing profile, default noise (`NOISE_PRESSURE = 2 Pa`) | State 6 first reported after true apogee, within 2.5 s of it and at most 15 m below true apogee altitude. Never before true apogee. |
| H2 | High baro noise | Pressure noise 10 Pa (about 0.9 m near 3 km) for the whole flight, several seeds (at least 20) | Same as H1 on every seed. No early trigger. |
| H3 | Single spike | One pressure sample equal to 40 m lower altitude during coast, then normal | No trigger caused by the spike. Nominal detection afterwards. |
| H4 | Short dip | 4 consecutive samples 15 m low during coast (one below the confirm count), then normal | No trigger. Repeat with 5 samples: trigger allowed, record it (documents the limit, not a pass or fail on its own). |
| H5 | Frozen barometer | Stop sending barometer packets (no new samples) from mid coast onward, plus a variant repeating the last value | B never fires. D fires: state 6 within 1 s of `APOGEE_TIMER_MS` after `BOOST` entry. |
| H6 | Stuck IMU | Accel kept at a boost like value after launch so `BOOST` never ends | Stays in `BOOST` (state 3) until D. State 6 within 1 s of `APOGEE_TIMER_MS`. B must not fire during `BOOST`. |
| H7 | Low apogee, no `ACTIVE_CONTROL` | `--apogee-alt` about 1500 m (below `COAST_ACTIVE_CONTROL_BAROM_ALT_THRESHOLD`) | State sequence `BOOST`, `COAST`, `APOGEE`, never state 5. B fires within 2.5 s of true apogee. |
| H8 | Apogee in `ACTIVE_CONTROL` | Nominal apogee above the gate | State sequence includes state 5, then 6 within 2.5 s of true apogee. Peak is not reset by the state change. |
| H9 | Transonic artefact in `BOOST` | Pressure jump equivalent to 100 m of altitude drop lasting 1 s during the burn | No trigger during `BOOST`. |
| H10 | Transonic artefact in `COAST` | Same jump lasting less than the confirm time, then lasting more | Short: no trigger. Long: trigger allowed, record it (known limit of B, D and the margin choice are the mitigation). |
| H11 | Timer as backstop only | Nominal rescaled profile, timer 28 s | D never fires before B in H1 to H4. If B fires first the run reports channel B. |
| H12 | Slow apogee | Long flat top: near zero vertical velocity for 5 s around apogee, then descent | No trigger during the plateau if it does not drop 7 m. Trigger after the descent. |
| H13 | Hard descent | Overshoot: fast fall right after apogee (for example 60 m/s) | Trigger within 1 s of the drop exceeding the margin. |
| H14 | Invalid pressure | Packets with NaN, +inf, 0 Pa, -1 Pa, 5 Pa, 200 kPa and a temperature of NaN or 500 C, single and in runs of 1 to 50 packets, during coast and again during the APOGEE state before 450 m | No drogue and no main chute caused by them. Detector still fires normally after the invalid run ends. In the APOGEE state the main chute must not fire at high altitude. |
| H15 | After apogee | Continue the profile to landing | Sequence `APOGEE`, `MAIN_PARACHUTE` at 450 m AGL, `LANDED` (landed check uses the windowed velocity, about 1 s confirm). No second drogue event. Also a run with a single 0 Pa or +100 m sample while descending at 450 to 600 m: the main chute must not fire early. |

Note H14: before this fix `CalculateAltitude` returned 0.0 for an invalid pressure, a false 3000 m drop. This scenario is the regression test for that fatal failure. Add the same cases for a spike UP (poisoning the peak): one sample equal to +500 m then normal, no trigger.

## 4. What to observe

- Telemetry `State` byte (name table in `hil.py`).
- Telemetry altitude, for the altitude error at detection.
- The trigger channel (B or D) is logged only, not in telemetry. Read it from the SD log after the run, or infer it from timing: D fires only at `APOGEE_TIMER_MS` after `BOOST` entry, B fires close to true apogee.
- The drogue pyro event, if the HIL setup exposes it, is only used to confirm one event per run.

## 5. Suggested run order

1. H1 and H11 to check the harness and the baseline.
2. H3, H4, H14 (false trigger guards).
3. H5, H6 (D backstop).
4. H7, H8 (state placement).
5. H2, H12, H13, H15, then H9 and H10.

Re-run everything when `APOGEE_DROP_M`, `APOGEE_CONFIRM_SAMPLES`, `APOGEE_TIMER_MS` or the target apogee (EuRoC rules) change.
