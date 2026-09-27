"""HIL scenario runner. Runs one scenario by ID or the whole suite.

    python run_all.py --port COM3
    python run_all.py --port COM3 --scenario S4
    python run_all.py --port /dev/ttyUSB0 --baud 115200 --scenario S1 S6

Scenarios that need the deep-calibration firmware (state 12, CalStatus) report
BLOCKED until WP0/WP-F land and harness/protocol.py is updated. S4 runs today.
"""

import argparse

from harness.link import Link
from scenarios import (
    s1_gesture_tumble,
    s2_pad_cal,
    s3_gyro_bias_regression,
    s4_flight_profile,
    s5_abort,
    s6_negative_cases,
    s7_rerun,
)

SCENARIOS = {
    "S1": s1_gesture_tumble,
    "S2": s2_pad_cal,
    "S3": s3_gyro_bias_regression,
    "S4": s4_flight_profile,
    "S5": s5_abort,
    "S6": s6_negative_cases,
    "S7": s7_rerun,
}


def main():
    parser = argparse.ArgumentParser(description="HIL scenario runner")
    parser.add_argument("--port", required=True, help="Serial port (COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--scenario", nargs="+", default=list(SCENARIOS),
                        help="Scenario IDs to run (default: all)")
    args = parser.parse_args()

    ids = [s.upper() for s in args.scenario]
    unknown = [s for s in ids if s not in SCENARIOS]
    if unknown:
        parser.error(f"unknown scenario(s): {', '.join(unknown)}")

    results = []
    with Link(args.port, args.baud) as link:
        for sid in ids:
            print(f"--- running {sid} ---")
            result = SCENARIOS[sid].run(link)
            print(result.report())
            results.append(result)

    print("\n=== summary ===")
    passed = blocked = failed = 0
    for res in results:
        if res.blocked:
            tag, blocked = "BLOCKED", blocked + 1
        elif res.passed:
            tag, passed = "PASS", passed + 1
        else:
            tag, failed = "FAIL", failed + 1
        print(f"  {res.scenario_id}: {tag}")
    print(f"\n{passed} passed, {failed} failed, {blocked} blocked")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
