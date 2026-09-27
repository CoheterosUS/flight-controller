"""S7: re-run. Run the tumble again with a different M_true; a second record is
appended, the newest wins, the older is kept until the sector is full. The append
and newest-wins behaviour is a WP-B flash test; over HIL the observable is that a
second full tumble succeeds and M stays valid after a reset."""

from harness import protocol as p
from harness import synth
from harness import waiters
from harness.result import ScenarioResult

from . import _common

SCENARIO_ID = "S7"


def run(link, cfg=None):
    cfg = cfg or _common.Config()
    r = ScenarioResult(SCENARIO_ID)
    if not _common.calstatus_available(link, r):
        return r

    M_true = synth.make_M_true(axis=(0.1, 0.9, 0.3), angle_deg=25.0,
                               scale=(0.97, 1.03, 1.0), skew=-0.02)
    _common.reset_board(link, cfg)
    if not _common.hold_gesture(link, r, cfg):
        return r
    if not _common.run_tumble(link, r, cfg, M_true):
        return r

    done = waiters.wait_for_state(link, p.STATE_IDLE, timeout_s=15.0)
    r.check(done is not None, "second tumble returned to IDLE")

    link.send_command(p.COMMAND_RESET)
    reloaded = waiters.wait_for_calstatus_bits(link, p.CALSTATUS_M_VALID, timeout_s=15.0)
    r.check(reloaded is not None, "newest M reloaded from flash after reset")
    return r
