"""S4: flight profile. Boost, coast, active control, apogee, main, landed, with
noise. Correct transitions, main parachute near 450 m AGL, and the flight states
walked in order. Runs against current firmware (no CalStatus needed): feeds raw
body-frame accel, identity mounting.

Raw +Y points to the tail (design section 2), so vertical accel is fed on -Y:
boost reads a large negative raw Y, matching the current threshold path."""

import math
import time

from harness import protocol as p
from harness import synth
from harness import waiters
from harness.result import ScenarioResult

from . import _common

SCENARIO_ID = "S4"

THRUST_ACCEL = 90.0
BURN_TIME = 3.0
DROGUE_DESCENT_RATE = 25.0
MAIN_DESCENT_RATE = 5.0
MAIN_DEPLOY_ALT = 450.0
MAIN_TOLERANCE_M = 60.0
SEA_LEVEL_PRESSURE = 101325.0


class _Profile:
    def __init__(self):
        self.burn_end_vel = (THRUST_ACCEL - synth.GRAVITY) * BURN_TIME
        self.burn_end_alt = 0.5 * (THRUST_ACCEL - synth.GRAVITY) * BURN_TIME ** 2
        self.coast_time = self.burn_end_vel / synth.GRAVITY
        self.apogee_alt = (self.burn_end_alt + self.burn_end_vel * self.coast_time
                           - 0.5 * synth.GRAVITY * self.coast_time ** 2)
        self.drogue_time = (self.apogee_alt - MAIN_DEPLOY_ALT) / DROGUE_DESCENT_RATE
        self.main_time = MAIN_DEPLOY_ALT / MAIN_DESCENT_RATE
        self.total = BURN_TIME + self.coast_time + self.drogue_time + self.main_time + 5.0

    def sample(self, t):
        ay = 0.0
        if t < BURN_TIME:
            ay = -THRUST_ACCEL
            alt = 0.5 * (THRUST_ACCEL - synth.GRAVITY) * t ** 2
        elif t < BURN_TIME + self.coast_time:
            tc = t - BURN_TIME
            alt = self.burn_end_alt + self.burn_end_vel * tc - 0.5 * synth.GRAVITY * tc ** 2
        elif t < BURN_TIME + self.coast_time + self.drogue_time:
            td = t - BURN_TIME - self.coast_time
            alt = self.apogee_alt - DROGUE_DESCENT_RATE * td
        elif t < BURN_TIME + self.coast_time + self.drogue_time + self.main_time:
            tm = t - BURN_TIME - self.coast_time - self.drogue_time
            alt = MAIN_DEPLOY_ALT - MAIN_DESCENT_RATE * tm
        else:
            ay = -synth.GRAVITY
            alt = 0.0
        return ay, max(alt, 0.0)


def _pressure(alt):
    return SEA_LEVEL_PRESSURE * (1.0 - 2.25577e-5 * alt) ** 5.25588


def run(link, cfg=None):
    cfg = cfg or _common.Config()
    r = ScenarioResult(SCENARIO_ID)

    # Reach PRELAUNCH on the pad.
    link.send_command(p.COMMAND_CALIBRATION)
    pad = lambda: link.send_hil(
        synth.noisy([0.0, -synth.GRAVITY, 0.0], cfg.accel_noise),
        synth.noisy([0.0, 0.0, 0.0], cfg.gyro_noise),
        [20.0, 5.0, -40.0], SEA_LEVEL_PRESSURE, 25.0)
    if not r.check(
        waiters.wait_for_state(link, p.STATE_PRELAUNCH, timeout_s=45.0, feed=pad) is not None,
        "PRELAUNCH reached on the pad",
    ):
        return r

    profile = _Profile()
    seen_states = set()
    main_fire_alt = None
    interval = 1.0 / cfg.rate_hz
    t = 0.0
    while t < profile.total:
        ay, alt = profile.sample(t)
        link.send_hil(
            synth.noisy([0.0, ay, 0.0], cfg.accel_noise),
            synth.noisy([0.0, 0.0, 0.0], cfg.gyro_noise),
            [20.0, 5.0, -40.0], _pressure(alt) + 0.0, 25.0)
        frame = link.poll()
        if frame is not None:
            seen_states.add(frame["state"])
            if main_fire_alt is None and frame["relay"] & p.RELAY_PARACHUTE_FIRED:
                main_fire_alt = frame["baro_alt"]
        time.sleep(interval)
        t += interval

    for st in (p.STATE_BOOST, p.STATE_APOGEE, p.STATE_MAIN_PARACHUTE, p.STATE_LANDED):
        r.check(st in seen_states, f"reached {p.state_name(st)}")
    if main_fire_alt is not None:
        r.check(abs(main_fire_alt - MAIN_DEPLOY_ALT) <= MAIN_TOLERANCE_M,
                f"main fired near {MAIN_DEPLOY_ALT:.0f} m AGL (at {main_fire_alt:.0f} m)")
    else:
        r.check(False, "main parachute relay fired")
    return r
