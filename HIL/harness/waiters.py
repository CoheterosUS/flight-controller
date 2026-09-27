"""Drive the board while waiting for a telemetry condition.

Each waiter pumps a feed() callback at the IMU rate (feed injects one sensor
sample), polls telemetry, and returns the frame that satisfied the predicate,
or None on timeout.
"""

import time

from . import protocol as p


def _run(link, feed, predicate, timeout_s, rate_hz):
    interval = 1.0 / rate_hz
    now = time.monotonic()
    deadline = now + timeout_s
    next_tick = now
    while time.monotonic() < deadline:
        if feed is not None:
            feed()
        frame = link.poll()
        if frame is not None and predicate(frame):
            return frame
        next_tick += interval
        slack = next_tick - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            next_tick = time.monotonic()  # behind schedule; don't accumulate lag
    return None


def wait_for_state(link, target, timeout_s, feed=None, rate_hz=p.IMU_ODR_HZ):
    targets = target if isinstance(target, (set, tuple, list)) else {target}
    return _run(link, feed, lambda f: f["state"] in targets, timeout_s, rate_hz)


def wait_for_pose(link, pose, timeout_s, feed=None, rate_hz=p.IMU_ODR_HZ):
    return _run(link, feed, lambda f: f.get("pose") == pose, timeout_s, rate_hz)


def wait_for_calstatus_bits(link, mask, timeout_s, feed=None, rate_hz=p.IMU_ODR_HZ):
    def pred(f):
        cs = f.get("calstatus")
        return cs is not None and (cs & mask) == mask
    return _run(link, feed, pred, timeout_s, rate_hz)


def wait_for_all_valid(link, timeout_s, feed=None, rate_hz=p.IMU_ODR_HZ):
    return wait_for_calstatus_bits(link, p.CALSTATUS_PRELAUNCH_MASK, timeout_s, feed, rate_hz)


def feed_for(link, feed, duration_s, rate_hz=p.IMU_ODR_HZ):
    """Inject samples for a fixed duration, polling telemetry. Return last frame."""
    interval = 1.0 / rate_hz
    now = time.monotonic()
    deadline = now + duration_s
    next_tick = now
    while time.monotonic() < deadline:
        feed()
        link.poll()
        next_tick += interval
        slack = next_tick - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            next_tick = time.monotonic()
    return link.last
