# Apogee detection: summary of changes (for the HIL colleague)

Branch `baro-apogee-fix`. Read this first, then `APOGEE_HIL.md` (scenarios) and `../APOGEE_DETECTION_PLAN.md` (full design). Firmware changes are being implemented in parallel; HIL work can start from the scenarios and the protocol changes below.

## Why

- The old trigger was `BarometricVelocity <= 0`, one sample, no confirmation. The velocity was recomputed every 4 ms but the barometer updates every 20 ms, so 4 of 5 iterations read exactly 0: a false drogue almost immediately after ACTIVE_CONTROL.
- Apogee could only be reached from ACTIVE_CONTROL (an EuRoC feature, entered above 2000 m).
- An invalid pressure became an altitude of 0 m: a false huge drop (false drogue) and `alt <= 450` (false main chute).

## Apogee detection logic (new)

- Two independent channels, no voting. The drogue fires when either is true (GPS and Kalman are NOT used, unvalidated).
  - **B, barometer drop from peak:** median-of-3 filtered altitude below `peak - 15 m` for 5 consecutive new valid barometer samples (about 100 ms at 50 Hz). Peak is the highest filtered altitude since launch. The median-of-3 is the only filter (removes isolated outliers, 20 ms lag). No derivative. BMP581 IIR and oversampling config unchanged.
  - **D, timer:** 28 s after BOOST entry, independent of every sensor.
- One detector, called from `HandleState`, runs in BOOST, COAST and ACTIVE_CONTROL:
  - B is active in COAST and ACTIVE_CONTROL (not in BOOST). Peak is tracked from launch and is not reset by state changes.
  - D is active in all three, so a stuck IMU (BOOST never ends) still gets a drogue.
  - ACTIVE_CONTROL contains no apogee code of its own and never gates apogee. Apogee works with or without it.
- No minimum altitude inhibit.
- Expected B latency: about 1.9 s after true apogee (15 m drop), so the drogue fires about 17 m below the peak.
- The trigger channel (baro, timer, command) is logged only (SD log, flash log if room), not in telemetry.

## Invalid barometer data

- A sample is ignored (not a drop, not a confirmation, not a landing sample) if: pressure is NaN, inf or <= 0; pressure outside 30 to 125 kPa; temperature non-finite or outside -40 to 85 C; pad reference invalid; nothing published yet or a failed read; derived altitude non-finite.
- Slew gate: a jump faster than `400 m/s * dt + 10 m` from the last accepted altitude is rejected too, so a spike up cannot poison the peak and a spike down cannot count as a drop.
- `BarometricAltitude` holds the last valid value instead of returning 0.

## Other firmware changes

- Barometer mailbox gets a 32-bit sample counter, so every consumer works on new samples only.
- Landing check fixed: barometric velocity is computed on new valid samples only, over a 1 s window (a raw 50 Hz derivative is far too noisy). The landed confirmation counts new valid samples (about 1 s).
- APOGEE to MAIN_PARACHUTE (450 m AGL) now needs 5 consecutive new valid samples (it had a single-sample check).
- COAST to ACTIVE_CONTROL confirmation counts sensor samples, not loop iterations.
- The old `ACTIVE_CONTROL_APOGEE_*` macros are removed.
- Manual `COMMAND_DROGUE` still works and is logged as trigger "command".

## What changes for HIL

- New command `COMMAND_HIL_BARO = 0x11`: two little endian floats (pressure Pa, temperature C), sent at **50 Hz**, one packet is one barometer sample.
- `COMMAND_HIL_DATA` (0x10) no longer carries pressure or temperature: 9 floats (accel, gyro, mag) at 100 Hz. `PROTOCOL.md` is updated with this.
- Default profile: apogee at **25 s** after launch, about 3000 m. B should fire at about 26.9 s, only 1.1 s before the 28 s timer. Add `--apogee-time`, `--apogee-alt`, `--scenario`, `--seed`, `--dry-run`.
- 15 scenarios (H1 to H15) with pass criteria in `APOGEE_HIL.md`. The most important ones: no early fire (H1, H2, H3, H4, H9), invalid pressure including 0 Pa and NaN (H14, this is the regression test for the fatal failure), timer backstop with a frozen or missing barometer or a stuck IMU (H5, H6), apogee without ACTIVE_CONTROL (H7), main chute must not fire early on a bad sample (H15).

## Parameters (`configuration.h`)

| Macro | Value |
|---|---|
| `APOGEE_DROP_M` | 15.0 |
| `APOGEE_CONFIRM_SAMPLES` | 5 |
| `APOGEE_TIMER_MS` | 28000 |
| `BARO_ODR_HZ` | 50 |
| `APOGEE_BARO_MAX_SPEED_MPS`, `APOGEE_BARO_SLEW_MARGIN_M` | 400, 10 |
| `BARO_VALID_MIN_PA`, `BARO_VALID_MAX_PA` | 30000, 125000 |
| `BARO_VALID_MIN_TEMP_C`, `BARO_VALID_MAX_TEMP_C` | -40, 85 |

## Open

- EuRoC rules will change the target apogee and the altitude gates: re-run all scenarios then (see `BACKLOG.md`).
- Confirm 28 s is later than the simulated apogee time.
