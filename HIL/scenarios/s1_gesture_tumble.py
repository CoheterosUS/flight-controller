"""S1: gesture and tumble. Hold nose-down 10 s, feed six poses from a known
M_true, expect DEEP_CALIBRATION, pose 1..6, M written, return to IDLE, and the
M still valid after a reset. (Exact M vs M_true is a WP-A host test, not HIL:
M is not in the telemetry packet.)"""

from harness import protocol as p
from harness import synth
from harness import waiters
from harness.result import ScenarioResult

from . import _common

SCENARIO_ID = "S1"


def run(link, cfg=None):
    cfg = cfg or _common.Config()
    r = ScenarioResult(SCENARIO_ID)
    if not _common.calstatus_available(link, r):
        return r

    M_true = synth.make_M_true(axis=(0.3, 0.4, 0.866), angle_deg=12.0,
                               scale=(1.02, 0.98, 1.01), skew=0.01)

    _common.reset_board(link, cfg)
    if not _common.hold_gesture(link, r, cfg):
        return r
    if not _common.run_tumble(link, r, cfg, M_true):
        return r

    done = waiters.wait_for_state(link, p.STATE_IDLE, timeout_s=15.0)
    r.check(done is not None, "returned to IDLE after tumble")
    frame = link.last or {}
    r.check(frame.get("calstatus", 0) & p.CALSTATUS_M_VALID, "M_VALID set in CalStatus")

    link.send_command(p.COMMAND_RESET)
    reloaded = waiters.wait_for_calstatus_bits(link, p.CALSTATUS_M_VALID, timeout_s=15.0)
    r.check(reloaded is not None, "M reloaded from flash after reset")
    return r
