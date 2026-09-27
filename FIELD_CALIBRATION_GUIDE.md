# Field calibration guide

For the person holding the rocket. Two separate procedures:

- **A. Deep calibration (tumble).** Done once per airframe or board, days before launch is fine. Produces the 12-parameter accelerometer matrix `M` and the gyro rotation, stored in flash. Takes about 5 minutes.
- **B. Pad calibration.** Done automatically at every power-up on the pad. Measures pressure reference, gyro bias and accel bias. Takes about 15 seconds.

Design background is in `DEEP_CALIBRATION_DESIGN.md`, the bring-up procedure in `BRINGUP_AND_HIL.md`. This document is the field checklist.

Status: written from the design and the firmware settings. The procedure has not yet been run on the real board. After the first hardware run, correct this guide with what you actually hear and see.

## 1. Before you start

- Flight build (`FLIGHT_BUILD 1`) or a HIL build makes no difference for the procedure. In a HIL build the sensors are simulated, so do this on real sensors only.
- Battery charged, board powered from the flight battery or a stable bench supply. Do not power cycle in the middle of a tumble.
- The rocket airframe is assembled as it will fly, with the board fixed in place. If the board moves relative to the airframe, `M` is no longer valid and you must redo A.
- Pyros disconnected or safed for the whole procedure (no ematches connected).
- Flat, stable surface for the pose holds, or a cradle. A stand that lets you hold each of six orientations without touching the rocket is best. Do not hold it in your hands for 30 seconds if you can avoid it: hand tremor triggers the motion check.
- Know your rocket axes. **+X is the nose. +Y and +Z are marked on the fuselage. If in doubt use the right-hand rule (X cross Y = Z).** A mirrored pose order is detected at the end and reported as a failure, but it wastes 5 minutes.
- No magnets, no vibrating machinery, no walking on the surface the rocket rests on.

## 2. Procedure A: deep calibration

### 2.1 Enter

1. Power up the board. Wait for the two short beeps (state machine started) and a short chirp when the state changes. Let it sit still for a few seconds.
2. Only in IDLE, CALIBRATION or PRELAUNCH does the gesture work. It is ignored in flight states and abort states.
3. Hold the rocket **nose down (ogive pointing at the ground), completely still, for 10 seconds.** Raw accel Y reads about +9.81 in this pose.
4. You hear **one long 2 second tone**. That means deep calibration is entered. If you hear nothing after 12 seconds, the hold was not accepted (moved too much, not vertical enough): put it upright, wait, and try again.

A serial command cannot start this. The gesture always works, so you can re-run the whole thing at any time, even from PRELAUNCH.

### 2.2 The six poses

After the entry tone the board prompts pose 1. **The number of beeps is the pose number** (each beep 0.4 s on, 0.4 s off). Move the rocket to the pose, then hold still.

| Prompt (beeps) | Pose | How to hold the rocket |
|---|---|---|
| 1 | +X | nose up (pointing at the sky) |
| 2 | -X | nose down |
| 3 | +Y | rocket on its side, the +Y mark pointing up |
| 4 | -Y | the +Y mark points down |
| 5 | +Z | the +Z mark points up |
| 6 | -Z | the +Z mark points down |

"Points up" means that axis is aligned with gravity (up), within about 20 degrees. The nose-up, nose-down poses have the fuselage vertical. For the four side poses lay the rocket on its side with the mark pointing exactly up or down, or use a cradle. Make sure each orientation is as close to an exact axis alignment as you can, because the accuracy of `M` depends on it.

Timing per pose (default settings):

1. Beep pattern plays. Move the rocket to the pose.
2. **Settle time, 10 s.** No data is used. Get it steady and let go.
3. **Sampling, 30 s.** Do not touch the rocket or the surface. Data is collected and checked.
4. If accepted, the next pose prompt plays. The whole sequence is about 4 to 5 minutes at the defaults.

Order matters. Keep it +X, -X, +Y, -Y, +Z, -Z: the sequence is how the board learns which hardware axis is which rocket axis.

### 2.3 If the prompt repeats

The same number of beeps again after about 40 seconds means the pose was rejected and is restarting:

- The rocket moved during sampling (gyro above 2 dps).
- The pose is not aligned with an axis (the gravity vector is not within about 20 degrees of the expected axis, or the total is not 1 g within 10 percent).
- The pose is not opposite (poses 2, 4, 6) or perpendicular (poses 3, 5) to the previous one as required. This is what a wrong orientation looks like, for example holding +Z where -Y was asked.

Reorient, get still, and wait for the new sampling window. After **5 restarts** on the run, the tumble fails. The whole tumble also has an overall timeout of about 8 minutes.

### 2.4 The result

| What you hear | Meaning | What to do |
|---|---|---|
| Three 1.2 second tones | Success: `M` solved, checked and stored in flash. State returns to IDLE. | Go to section 4 (verify). |
| One 4 second tone | Failure. Nothing was stored. State returns to IDLE (a hardware fault goes to GROUND_ABORT instead). | Section 5 (troubleshooting), then repeat from 2.1. |

A failure never overwrites the calibration already in flash. If you had a good `M` before, you still have it.

## 3. Procedure B: pad calibration (every power-up)

Automatic. Do this at the launch rail, with the rocket in its flight position.

1. Stand the rocket **nose straight up, on the rail, completely still.** Wind shake, people leaning on the rail, and handling all restart the measurement.
2. Power up (or leave it powered, if the board is already in CALIBRATION).
3. The board measures for about 15 seconds: pressure reference (about 6 s), gyro bias, accel bias. Any motion or a clear tilt restarts the measurement window.
4. When everything is valid it moves to PRELAUNCH.

Things to know:

- **Nose up is required.** The accel bias is taken with gravity along +X, on all three axes. A rocket that is tilted has its lateral tilt read as a false bias (an accepted compromise, see the design document). A gross tilt (beyond about 0.1 g lateral) is rejected: the measurement never completes, and you stay in CALIBRATION. Set the rail closer to vertical, or if the rail is intentionally angled, accept that the bias includes the tilt.
- **The board will not enter PRELAUNCH without a valid `M`.** With no calibration in flash it sits in CALIBRATION indefinitely and never arms. The way out is procedure A.
- If you re-run procedure A on the pad, the pad calibration re-runs afterwards.
- The bias is measured every boot. It is not stored: do not power cycle after PRELAUNCH unless you want to redo it.

## 4. Verify (do this after every successful tumble)

What the telemetry shows. A flight build (`HIL_MODE 0`) sends the original 52 byte packet (`PROTOCOL.md`, "Wire Telemetry Packet") at 1 Hz. It does **not** contain `CalStatus`, so the individual calibration bits and the tumble pose are not visible on the link. What you can read:

- `State` (offset 48): 0 IDLE, 1 CALIBRATION, 2 PRELAUNCH, 12 DEEP_CALIBRATION (the tumble is running), 9 GROUND_ABORT.
- `Flags` (offset 42): the fault bitmask.
- The accel and gyro fields (offsets 6 to 16, `CalAccelX` to `CalGyroZ`): calibrated body-frame values, truncated to integers. Older ground software may label them `AccelX` to `GyroZ`, the bytes are the same.
- The packet length itself: 52 bytes means a non-HIL build. A HIL build sends 54 bytes.

Bit level detail (`CalStatus`) needs either a HIL build (`CalStatus` is in its 54 byte telemetry, but the sensors are simulated there, so it cannot verify a real board) or the SD log (`CalStatus` at offset 184 of every record, only written when the build has `SD_LOGGING_ENABLED 1`, which is off by default). The flash log does not carry `CalStatus`.

1. Power cycle the board. `State` goes to 1 (CALIBRATION). `Flags` zero.
2. Hold the rocket nose up and still. Check the calibrated readings in telemetry:
   - `CalAccelX` about +9.8 (int16 in the wire packet, so 9 or 10), `CalAccelY` and `CalAccelZ` 0 or +-1.
   - `CalGyro` all 0.
   Values appear truncated to integers on the telemetry link. The fine calibrated values are in the SD log (float, when SD logging is enabled). The flash log only holds raw hardware-axis accel and gyro, not the calibrated values.
3. Rotate slowly about the nose axis and watch `CalGyroX`. Turn the rocket nose-up to a horizontal position with the +Y mark up: `CalAccelY` should go to about +9.8, `CalAccelX` to about 0. Readings that follow the rocket axes like this show that the stored `M` loaded. Without a valid `M` these fields carry the raw hardware-axis values instead (with the board's default mounting, nose up reads about -9.8 on `CalAccelY` and about 0 on `CalAccelX`).
4. Stand it nose up and still, and wait for `State` 2 (PRELAUNCH). The board only enters PRELAUNCH when `M`, the gyro bias, the accel bias and the pressure reference are all valid (`CalStatus` bits 0 to 3), so reaching state 2 is the proof on a flight build. `Flags` zero. HIL mode and pre-seed are off by construction: `FLIGHT_BUILD 1` refuses to compile with either one, and the 52 byte packet confirms `HIL_MODE 0`.
5. Optional bench check: read the calibration record from flash with the flash dump tool. Sequence number increased by one, CRC good, `det(Q)` about +1.

## 5. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| No 2 s tone after 12 s nose down | Not held still, not vertical, or not in IDLE, CALIBRATION or PRELAUNCH (for example the board is in a flight or abort state). | Put the rocket upright, wait 5 s, hold nose down again. Check the state in telemetry. |
| Entry tone plays, then no prompt for a while | The first prompt follows the entry tone after a short delay. | Keep the rocket still, nose down, until pose 1 is prompted. |
| A pose repeats forever | Rocket not still or not aligned, or the prompt is being misread. | Clamp the rocket in a cradle. Count the beeps again. Check the mark you are pointing up. |
| Fails with the long 4 s tone after all six poses | Mirrored pose order (right-hand rule violated), a pose held wrongly but accepted, or a bad sensor. The solve rejects `det(Q)` near -1, big residuals or a large per-pose error. | Check where the +Y and +Z marks really are. Re-run, keeping the order and the marks straight. If it fails twice with correct poses, suspect the sensor: check accel noise in telemetry. |
| Fails at once with the 4 s tone | A flash or sensor fault. The state goes to GROUND_ABORT, not IDLE. | Read `Flags` (`PROTOCOL.md`, fault table): bits 10 to 14 are flash faults, bits 0 to 7 sensor faults. Fix, power cycle. |
| Stays in CALIBRATION forever after a good tumble | Pad measurement is restarting: rocket moving, not nose up, or a bias out of limit. | Stand it nose up and still. The flight telemetry cannot say which item is missing (`CalStatus` bits 1, 2, 3 are not on the 52 byte packet): read them from the SD log if it is enabled. |
| Never leaves CALIBRATION, even nose up and still | No valid `M` in flash (`CalStatus` bit 0 clear, visible only in the SD log), or the case above. The accel readings in step 4.3 not following the rocket axes points to a missing `M`. | Run procedure A. |
| `Flags` bit 12 (LOG_FULL) after boot | The flight log region is full. | Erase the log before flight (`FLASH_ERASE_ALL` maintenance build). The calibration sectors are preserved. |
| Bit 13 or 14 set | Flash write failed, or the flight snapshot failed to verify. | Do not fly. Re-check the flash, power cycle, retry. |
| Buzzer silent | Buzzer disabled or wiring. | The pose number (`CalStatus` bits 8 to 10) is not on the flight telemetry. You can see `State` 12 while the tumble runs and the return to 0 (IDLE) at the end, or 9 (GROUND_ABORT) on a fault. Without the buzzer, time the poses (10 s settle plus 30 s sample each) or fix the buzzer first. The pose field is visible only in a HIL build or the SD log. |

## 6. When to redo procedure A

- The board was moved, remounted or the airframe was rebuilt.
- After a hard landing or anything that could have shifted the board.
- The `M` matrix looks wrong at the verify step.
- Before an important flight if the last tumble is old and the conditions (temperature, board changes) differ.

Not needed between flights if the board has not moved: `M` stays in flash. Procedure B (pad) runs every time on its own.

## 7. Rules for launch day

- The pad calibration is the last step before arming. Do not run procedure A on the rail unless you must: it takes 5 minutes and needs a still, handled rocket.
- **Never do the nose-down gesture in PRELAUNCH on the rail with the ematches connected.** The gesture is only accepted in pre-flight states and does not fire pyros, but keep pyros safed for any handling.
- Write down for each board: date of the last tumble, the calibration sequence number, and the `CalAccel` and `CalGyro` readings at the verify step. Compare after a re-run: `M` should not change much between two tumbles of the same board (a few tenths of a percent per element). A big jump is a warning.
- A flight build with `FLIGHT_BUILD 1` has serial commands off. There is no way to force state changes. The only ways in and out of states are the sensors and the gesture.

## 8. Quick reference

| Item | Value |
|---|---|
| Gesture | nose down, still, 10 s (raw accel Y about +9.81) |
| Where it works | IDLE, CALIBRATION, PRELAUNCH |
| Entry | one 2 s tone |
| Pose prompt | pose number in beeps (0.4 s on, 0.4 s off) |
| Poses | 1 +X, 2 -X, 3 +Y, 4 -Y, 5 +Z, 6 -Z (nose is +X; +Y and +Z are the fuselage marks) |
| Per pose | 10 s settle then 30 s sample, no touching |
| Restarts allowed | 5 per run |
| Success | three 1.2 s tones, back to IDLE |
| Failure | one 4 s tone, back to IDLE (a fault goes to GROUND_ABORT) |
| Storage | two flash sectors at `0x003FE000` and `0x003FF000`, append only, newest wins |
| Pad calibration | nose up, still, about 15 s, automatic every boot |
| Needed for PRELAUNCH | valid `M`, gyro bias, accel bias, pressure reference (`CalStatus` bits 0 to 3, SD log or HIL telemetry only) |
| Flight telemetry | 52 bytes at 1 Hz: `State`, `Flags`, truncated `CalAccel` and `CalGyro`, no `CalStatus` |
