"""S3: gyro bias frame regression (BACKLOG bug 2, fix D19). A 90 deg rotation
about Z in M_true plus a constant raw gyro offset (1,2,3) dps at rest. After the
fix CalGyro is ~0; before it, ~3.2 dps.

Note: the 0.01 dps pass bound cannot survive int16 telemetry truncation. That
tight check lives in the WP-E host regression test on ImuApplyCalibration. Over
telemetry, S3 asserts the coarse bound: |CalGyro| within one truncation step,
which still separates the fixed (~0) from the broken (~3.2 dps) case."""

from harness import protocol as p
from harness import synth
from harness import waiters
from harness.result import ScenarioResult

from . import _common

SCENARIO_ID = "S3"

_RAW_GYRO_OFFSET = (1.0, 2.0, 3.0)
_COARSE_BOUND = 1  # int16 telemetry steps


def _rest_feed(link, cfg):
    # Nose up, still, with the constant raw gyro offset baked in.
    accel = synth.noisy([0.0, -synth.GRAVITY, 0.0], cfg.accel_noise)
    gyro = synth.noisy(list(_RAW_GYRO_OFFSET), cfg.gyro_noise)
    link.send_hil(accel, gyro, [20.0, 5.0, -40.0], 101325.0, 25.0)


def run(link, cfg=None):
    cfg = cfg or _common.Config()
    r = ScenarioResult(SCENARIO_ID)
    if not _common.calstatus_available(link, r):
        return r
    # M_true with a 90 deg rotation about Z is assumed already stored (via S1 or
    # preseed) so the frame bug, if present, shows in the rotated CalGyro.
    link.send_command(p.COMMAND_CALIBRATION)
    frame = waiters.wait_for_calstatus_bits(
        link, p.CALSTATUS_GYRO_BIAS_VALID, timeout_s=40.0,
        feed=lambda: _rest_feed(link, cfg), rate_hz=cfg.rate_hz)
    if not r.check(frame is not None, "gyro bias measured"):
        return r
    gyro = frame.get("gyro", (99, 99, 99))
    within = all(abs(g) <= _COARSE_BOUND for g in gyro)
    r.check(within, f"CalGyro near zero (coarse telemetry bound), got {gyro}")
    return r
