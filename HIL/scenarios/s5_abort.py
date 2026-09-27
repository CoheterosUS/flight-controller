"""S5: abort. Trigger a fault during PRELAUNCH, expect GROUND_ABORT, pyros safed,
Kalman stops. A fault flag is provoked by a GROUND_ABORT command (HIL has no way
to force a sensor-mode fault); the state and safed relays are the observable
pass criteria over telemetry."""

from harness import protocol as p
from harness import synth
from harness import waiters
from harness.result import ScenarioResult

from . import _common

SCENARIO_ID = "S5"


def run(link, cfg=None):
    cfg = cfg or _common.Config()
    r = ScenarioResult(SCENARIO_ID)

    link.send_command(p.COMMAND_CALIBRATION)
    pad = lambda: link.send_hil(
        synth.noisy([0.0, -synth.GRAVITY, 0.0], cfg.accel_noise),
        synth.noisy([0.0, 0.0, 0.0], cfg.gyro_noise),
        [20.0, 5.0, -40.0], 101325.0, 25.0)
    if not r.check(
        waiters.wait_for_state(link, p.STATE_PRELAUNCH, timeout_s=45.0, feed=pad) is not None,
        "PRELAUNCH reached",
    ):
        return r

    link.send_command(p.COMMAND_GROUND_ABORT)
    frame = waiters.wait_for_state(link, p.STATE_GROUND_ABORT, timeout_s=10.0, feed=pad)
    if not r.check(frame is not None, "GROUND_ABORT entered"):
        return r
    fired = frame["relay"] & (p.RELAY_DROGUE_FIRED | p.RELAY_PARACHUTE_FIRED)
    r.check(fired == 0, "pyros safed (no relay bits set)")
    return r
