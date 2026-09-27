"""S6: negative cases. Each sub-case is its own set of checks, with a reset
between them so one does not poison the next. Firmware-dependent ones self-block
until WP0/WP-F land.

Two limits to be honest about, both needing a firmware hook (not done here):
  - "no M" needs an ERASED calibration sector. Reset does not erase it (D18), so
    this sub-case only truly exercises a fresh board; on a board that already has
    an M it will (correctly) reach PRELAUNCH. Flagged in the check message.
  - success vs failure both end in IDLE. Without a deep-cal result code in
    CalStatus, failure is inferred as "aborted before pose 6", not read directly.
"""

from harness import protocol as p
from harness import synth
from harness import waiters
from harness.result import ScenarioResult

from . import _common

SCENARIO_ID = "S6"

_M_IDENT = synth.make_M_true()


def run(link, cfg=None):
    cfg = cfg or _common.Config()
    r = ScenarioResult(SCENARIO_ID)
    if not _common.calstatus_available(link, r):
        return r

    _no_M_refuses_prelaunch(link, r, cfg)
    _mirrored_order_rejected(link, r, cfg)
    _motion_restarts_pose(link, r, cfg)
    _wrong_way_restarts_pose(link, r, cfg)
    _continuous_motion_aborts(link, r, cfg)
    _gesture_in_boost_ignored(link, r, cfg)
    return r


def _no_M_refuses_prelaunch(link, r, cfg):
    _common.reset_board(link, cfg)
    link.send_command(p.COMMAND_CALIBRATION)
    pad = lambda: link.send_hil(
        synth.noisy([0.0, -synth.GRAVITY, 0.0], cfg.accel_noise),
        synth.noisy([0.0, 0.0, 0.0], cfg.gyro_noise),
        [20.0, 5.0, -40.0], 101325.0, 25.0)
    reached = waiters.wait_for_state(link, p.STATE_PRELAUNCH, timeout_s=20.0, feed=pad)
    r.check(reached is None,
            "no M: PRELAUNCH refused (only valid on a board with no stored M)")


def _mirrored_order_rejected(link, r, cfg):
    _common.reset_board(link, cfg)
    if not _common.hold_gesture(link, r, cfg):
        return
    _common.run_tumble(link, r, cfg, _M_IDENT, pose_order=[2, 1, 3, 4, 5, 6])
    idle = waiters.wait_for_state(link, p.STATE_IDLE, timeout_s=15.0)
    frame = link.last or {}
    r.check(idle is not None and not (frame.get("calstatus", 0) & p.CALSTATUS_M_VALID),
            "mirrored order: no valid M written (det(Q) numeric is a WP-A host test)")


def _motion_restarts_pose(link, r, cfg):
    _common.reset_board(link, cfg)
    if not _common.hold_gesture(link, r, cfg):
        return
    if not waiters.wait_for_pose(link, 1, timeout_s=cfg.pose_timeout_s,
                                 feed=lambda: _common.feed_pose(link, cfg, _M_IDENT, 1)):
        r.check(False, "motion: pose 1 reached")
        return
    # feed pose-1 orientation but shaking (gyro above the motion limit)
    advanced = waiters.wait_for_pose(
        link, 2, timeout_s=cfg.observe_s,
        feed=lambda: _common.feed_pose(link, cfg, _M_IDENT, 1,
                                       gyro=(cfg.motion_gyro_dps, 0.0, 0.0)))
    r.check(advanced is None, "motion during sample: pose did not advance")


def _wrong_way_restarts_pose(link, r, cfg):
    _common.reset_board(link, cfg)
    if not _common.hold_gesture(link, r, cfg):
        return
    if not waiters.wait_for_pose(link, 1, timeout_s=cfg.pose_timeout_s,
                                 feed=lambda: _common.feed_pose(link, cfg, _M_IDENT, 1)):
        r.check(False, "wrong-way: pose 1 reached")
        return
    # prompt is pose 1 (+X) but we feed pose 2 (-X) orientation
    advanced = waiters.wait_for_pose(
        link, 2, timeout_s=cfg.observe_s,
        feed=lambda: _common.feed_pose(link, cfg, _M_IDENT, 2))
    r.check(advanced is None, "wrong-way pose: pose did not advance")


def _continuous_motion_aborts(link, r, cfg):
    _common.reset_board(link, cfg)
    if not _common.hold_gesture(link, r, cfg):
        return
    # shake through every pose so restarts/timeout are exhausted
    saw_pose_6 = waiters.wait_for_pose(
        link, 6, timeout_s=cfg.abort_timeout_s,
        feed=lambda: _common.feed_pose(link, cfg, _M_IDENT, 1,
                                       gyro=(cfg.motion_gyro_dps, 0.0, 0.0)))
    idle = waiters.wait_for_state(link, p.STATE_IDLE, timeout_s=cfg.abort_timeout_s)
    r.check(saw_pose_6 is None and idle is not None,
            "continuous motion: aborted to IDLE without completing pose 6")


def _gesture_in_boost_ignored(link, r, cfg):
    _common.reset_board(link, cfg)
    boost = lambda: link.send_hil(
        synth.noisy([0.0, -90.0, 0.0], cfg.accel_noise),
        synth.noisy([0.0, 0.0, 0.0], cfg.gyro_noise),
        [20.0, 5.0, -40.0], 101325.0, 25.0)
    waiters.wait_for_state(link, p.STATE_BOOST, timeout_s=45.0, feed=boost)
    entered = waiters.wait_for_state(
        link, p.STATE_DEEP_CALIBRATION, timeout_s=cfg.hold_s + 5.0,
        feed=lambda: _common.gesture_feed(link, cfg))
    r.check(entered is None, "gesture in BOOST ignored")
