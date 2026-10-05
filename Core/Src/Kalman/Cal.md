# Six-Point Calibration Integration

## Rocket Body Frame
- +X = nose
- Y, Z = lateral (arbitrary assignment)
- Accelerometer at rest: axis pointing up reads +g

## Face Ordering (DeepCalibrationStateHandler steps)

Activation: hold nose down and still. Then move it; sampling starts at step 0.

| Step | Physical position | Axis pointing up | six_point_cal Y block |
|------|-------------------|------------------|-----------------------|
| 0 | Nose up | +X | +9.8 in X |
| 1 | Nose down | -X | -9.8 in X |
| 2 | Horizontal | +Y | +9.8 in Y |
| 3 | 180 from horizontal | -Y | -9.8 in Y |
| 4 | 90 from horizontal | +Z | +9.8 in Z |
| 5 | 180 from that | -Z | -9.8 in Z |

Order matches six_point_cal Y as written. No reorder needed.

Step 4 roll direction must be consistent (Z = X cross Y), otherwise A_m is a
reflection (det = -1) instead of a rotation.

---

A_m and M need to be stored to rotate the gyro too
