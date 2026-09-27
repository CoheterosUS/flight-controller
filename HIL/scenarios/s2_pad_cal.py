"""S2: pad calibration. Static nose-up with a known accel/gyro bias, an M
already valid. Expect all CalStatus validity bits set and PRELAUNCH reached.
Assumes S1 (or a preseed) has produced a valid M first."""

from harness import protocol as p
from harness import synth
from harness import waiters
from harness.result import ScenarioResult

from . import _common

SCENARIO_ID = "S2"

# Nose up: raw AccelY ≈ -9.81 (raw +Y toward tail, design section 2).
_GYRO_BIAS = (0.5, -0.3, 0.2)


def _pad_feed(link, cfg):
    accel = synth.noisy([0.0, -synth.GRAVITY, 0.0], cfg.accel_noise)
    gyro = synth.noisy(list(_GYRO_BIAS), cfg.gyro_noise)
    link.send_hil(accel, gyro, [20.0, 5.0, -40.0], 101325.0, 25.0)


def run(link, cfg=None):
    cfg = cfg or _common.Config()
    r = ScenarioResult(SCENARIO_ID)
    if not _common.calstatus_available(link, r):
        return r

    link.send_command(p.COMMAND_CALIBRATION)
    all_valid = waiters.wait_for_all_valid(
        link, timeout_s=40.0, feed=lambda: _pad_feed(link, cfg), rate_hz=cfg.rate_hz)
    r.check(all_valid is not None, "all CalStatus validity bits set")

    prelaunch = waiters.wait_for_state(
        link, p.STATE_PRELAUNCH, timeout_s=15.0,
        feed=lambda: _pad_feed(link, cfg), rate_hz=cfg.rate_hz)
    r.check(prelaunch is not None, "PRELAUNCH reached")
    return r
