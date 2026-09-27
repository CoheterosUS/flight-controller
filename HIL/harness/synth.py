"""Synthetic sensor generation shared across scenarios.

M_true is the ground-truth accelerometer map the board must recover:
    CalAccel = M * RawAccel - AccelBiasCal   (design D1/D20)
so to feed a pose whose ideal body reading is `cal`, we inject
    RawAccel = M_true^-1 * (cal + bias).
Keep the M_true model (rotation, scale, skew, offset, noise) in step with the
WP-A host-test fixtures so a solved-M mismatch flags one bug, not two suites.
"""

import math
import random

GRAVITY = 9.81


def mat_vec(M, v):
    return [
        M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
        M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
        M[6] * v[0] + M[7] * v[1] + M[8] * v[2],
    ]


def mat_mul(A, B):
    out = [0.0] * 9
    for r in range(3):
        for c in range(3):
            out[r * 3 + c] = sum(A[r * 3 + k] * B[k * 3 + c] for k in range(3))
    return out


def mat_inv(M):
    a, b, c, d, e, f, g, h, i = M
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError("singular matrix")
    inv_det = 1.0 / det
    return [
        (e * i - f * h) * inv_det,
        (c * h - b * i) * inv_det,
        (b * f - c * e) * inv_det,
        (f * g - d * i) * inv_det,
        (a * i - c * g) * inv_det,
        (c * d - a * f) * inv_det,
        (d * h - e * g) * inv_det,
        (b * g - a * h) * inv_det,
        (a * e - b * d) * inv_det,
    ]


def rotation_axis_angle(axis, angle_rad):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    t = 1.0 - c
    return [
        t * x * x + c, t * x * y - s * z, t * x * z + s * y,
        t * x * y + s * z, t * y * y + c, t * y * z - s * x,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c,
    ]


def make_M_true(axis=(0, 0, 1), angle_deg=0.0, scale=(1.0, 1.0, 1.0), skew=0.0):
    """Rotation * scale * (I + skew). Identity when called with defaults."""
    R = rotation_axis_angle(axis, math.radians(angle_deg))
    S = [scale[0], skew, 0.0, 0.0, scale[1], skew, 0.0, 0.0, scale[2]]
    return mat_mul(R, S)


IDENTITY_M = make_M_true()

# Ideal body-frame (Cal) accel for each tumble pose, forward-right-down, +g up
# (design section 2). Pose 1..6 = +X,-X,+Y,-Y,+Z,-Z.
_POSE_CAL = {
    1: (+GRAVITY, 0.0, 0.0),
    2: (-GRAVITY, 0.0, 0.0),
    3: (0.0, +GRAVITY, 0.0),
    4: (0.0, -GRAVITY, 0.0),
    5: (0.0, 0.0, +GRAVITY),
    6: (0.0, 0.0, -GRAVITY),
}


def raw_accel_for_pose(pose, M_true, accel_bias_cal=(0.0, 0.0, 0.0)):
    """Raw accel to inject so the board sees the ideal Cal reading for `pose`."""
    cal = _POSE_CAL[pose]
    biased = [cal[i] + accel_bias_cal[i] for i in range(3)]
    return mat_vec(mat_inv(M_true), biased)


def noisy(vec, sigma):
    return [v + random.gauss(0.0, sigma) for v in vec]
