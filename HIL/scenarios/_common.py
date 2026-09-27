"""Shared building blocks for the deep-calibration scenarios."""

import time

from harness import protocol as p
from harness import synth
from harness import waiters


class Config:
    """Scenario knobs. Mirror the DEEP_CAL_* macros WP0 adds to configuration.h;
    values here are the design defaults (section 3, all marked DEFAULT)."""

    def __init__(self):
        self.hold_s = 10.0          # DEEP_CAL_HOLD_MS
        self.settle_s = 10.0        # DEEP_CAL_POSE_SETTLE_MS
        self.sample_s = 30.0        # DEEP_CAL_POSE_SAMPLE_MS
        self.pose_timeout_s = 60.0  # per-pose wait for the pose to advance
        self.enter_timeout_s = 15.0
        self.abort_timeout_s = 120.0  # bound on the failure path (restarts/timeout)
        self.rate_hz = p.IMU_ODR_HZ
        self.accel_noise = 0.02
        self.gyro_noise = 0.005
        self.motion_gyro_dps = 20.0  # well above DEEP_CAL_GYRO_MOTION_MAX_DPS (2)
        self.observe_s = 20.0        # window to confirm a pose does NOT advance


def calstatus_available(link, result, probe_s=2.0):
    """True if telemetry carries CalStatus. Otherwise mark the result blocked."""
    if not p.CALSTATUS_PRESENT:
        result.block(
            "firmware does not write CalStatus into the battery slot yet "
            "(WP0/WP-F not landed); set CALSTATUS_PRESENT in harness/protocol.py once it does"
        )
        return False
    deadline = time.monotonic() + probe_s
    while time.monotonic() < deadline:
        frame = link.poll()
        if frame is not None and frame.get("calstatus") is not None:
            return True
        time.sleep(0.02)
    result.block("no telemetry with CalStatus received")
    return False


def reset_board(link, cfg, timeout_s=10.0):
    """Send RESET and wait for a pre-flight state (IDLE or CALIBRATION), so each
    scenario starts from a known point. Cannot erase a stored M: the calibration
    sector survives reset by design (D18), so this does NOT give a no-M board."""
    link.send_command(p.COMMAND_RESET)
    return waiters.wait_for_state(
        link, {p.STATE_IDLE, p.STATE_CALIBRATION}, timeout_s,
        feed=lambda: gesture_feed(link, cfg), rate_hz=cfg.rate_hz)


def gesture_feed(link, cfg, noise=True):
    """One nose-down still sample: raw AccelY ≈ +g, gyro ≈ 0 (design D8)."""
    accel = [0.0, synth.GRAVITY, 0.0]
    gyro = [0.0, 0.0, 0.0]
    if noise:
        accel = synth.noisy(accel, cfg.accel_noise)
        gyro = synth.noisy(gyro, cfg.gyro_noise)
    link.send_hil(accel, gyro, [20.0, 5.0, -40.0], 101325.0, 25.0)


def hold_gesture(link, result, cfg):
    """Feed the 10 s nose-down hold, expect entry into DEEP_CALIBRATION."""
    frame = waiters.wait_for_state(
        link, p.STATE_DEEP_CALIBRATION,
        timeout_s=cfg.hold_s + cfg.enter_timeout_s,
        feed=lambda: gesture_feed(link, cfg),
        rate_hz=cfg.rate_hz,
    )
    return result.check(frame is not None, "gesture entered DEEP_CALIBRATION (state 12)")


def run_tumble(link, result, cfg, M_true, accel_bias=(0.0, 0.0, 0.0), pose_order=range(1, 7)):
    """Step the six poses, feeding the raw accel each pose should produce.
    Returns True if all six poses were reached and sampled."""
    ok = True
    for pose in pose_order:
        reached = waiters.wait_for_pose(
            link, pose, timeout_s=cfg.pose_timeout_s,
            feed=lambda pth=pose: feed_pose(link, cfg, M_true, pth, accel_bias),
            rate_hz=cfg.rate_hz,
        )
        if not result.check(reached is not None, f"pose {pose} prompt reached"):
            ok = False
            break
        # settle + sample window: keep feeding the pose's raw data
        waiters.feed_for(
            link, lambda pth=pose: feed_pose(link, cfg, M_true, pth, accel_bias),
            duration_s=cfg.settle_s + cfg.sample_s, rate_hz=cfg.rate_hz,
        )
    return ok


def feed_pose(link, cfg, M_true, pose, accel_bias=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0)):
    """One raw sample for `pose`. gyro override lets a caller inject motion."""
    accel = synth.noisy(synth.raw_accel_for_pose(pose, M_true, accel_bias), cfg.accel_noise)
    gyro = synth.noisy(list(gyro), cfg.gyro_noise)
    link.send_hil(accel, gyro, [20.0, 5.0, -40.0], 101325.0, 25.0)
