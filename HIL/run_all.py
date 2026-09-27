#!/usr/bin/env python3
"""HIL suite runner. Runs one scenario by ID or the whole suite, on top of hil.py.

    python run_all.py --port COM3                    # S1 to S7 and R1, in order
    python run_all.py --port COM3 --scenario S4      # one
    python run_all.py --port COM3 --scenario S1 S6   # a subset
    python run_all.py --port COM3 --scenario APOGEE  # apogee scenarios H1 to H15
    python run_all.py --port sim                     # self test against the mock board

Each entry is one or more hil.py runs (see HIL/HIL_TEST_PLAN.md for the pass criteria). Result per scenario:
PASS, FAIL, or MANUAL (needs a manual step that the harness cannot do over the UART: erasing sectors,
provoking a fault, dumping flash). Exit code 1 if anything failed.

Suite runner and scenario IDs by Angelo Sho Moraschi (branch hil). The scenario bodies live in hil.py, which
speaks the current protocol (54 byte HIL telemetry with CalStatus, COMMAND_HIL_DATA plus COMMAND_HIL_BARO).
The board must run a HIL build: a flight build (52 byte telemetry) is refused by hil.py with a SETUP ERROR.
"""

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HIL_PY = os.path.join(HERE, "hil.py")

APOGEE = ["H1", "H2", "H3", "H3up", "H4", "H4b", "H5", "H5b", "H6", "H7", "H8", "H9", "H10", "H10b",
          "H12", "H13", "H14", "H15"]

# id -> (description, [list of hil.py argument lists]) or (description, None) for a manual scenario
SCENARIOS = {
    "S1": ("gesture and tumble, then pad calibration", [["--scenario", "calibrate"]]),
    "S2": ("pad calibration", [["--scenario", "pad"]]),
    "S3": ("gyro bias frame regression (90 deg roll, raw offset 1, 2, 3 dps)",
           [["--scenario", "calibrate", "--mounting", "roll90"]]),
    "S4": ("nominal flight profile", [["--scenario", "flight"]]),
    "S4b": ("pad wait (10 minutes)", [["--scenario", "pad_wait", "--duration", "600"]]),
    "S5": ("abort during PRELAUNCH and ascent (needs a fault provocation)", None),
    "S6": ("negative tumble cases (start from erased calibration sectors)",
           [["--scenario", "tumble", "--mirrored"],
            ["--scenario", "tumble", "--motion-pose", "3"],
            ["--scenario", "tumble", "--wrong-pose", "3"],
            ["--scenario", "tumble", "--restart-forever", "2"]]),
    "S7": ("re-run with a different M", [["--scenario", "regesture", "--mounting", "alt"]]),
    "R1": ("serial state commands are refused", [["--scenario", "commands"],
                                                ["--scenario", "flight", "--drogue-cmd-at", "1.0"]]),
    "R8": ("NaN and inf in the IMU during flight", [["--scenario", "flight", "--nan-imu-at", "12",
                                                     "--nan-imu-count", "20"]]),
    "APOGEE": ("apogee scenarios H1 to H15", [["--scenario", "flight", "--hid", h] for h in APOGEE]),
}
DEFAULT_ORDER = ["S1", "S2", "S3", "S4", "S4b", "S5", "S6", "S7", "R1", "R8"]


def main():
    parser = argparse.ArgumentParser(description="HIL suite runner")
    parser.add_argument("--port", required=True, help="Serial port (COM3, /dev/ttyUSB0) or 'sim'")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--scenario", nargs="+", default=DEFAULT_ORDER,
                        help="Scenario IDs to run (default: S1 to S7, R1, R8). APOGEE runs H1 to H15")
    parser.add_argument("--list", action="store_true", help="List the scenario IDs and exit")
    args = parser.parse_args()

    if args.list:
        for sid, (desc, runs) in SCENARIOS.items():
            print(f"{sid:7s} {desc}{'' if runs else '  [manual]'}")
        return 0

    ids = [s if s in SCENARIOS else s.upper() for s in args.scenario]
    unknown = [s for s in ids if s not in SCENARIOS]
    if unknown:
        parser.error(f"unknown scenario(s): {', '.join(unknown)}")

    summary = []
    for sid in ids:
        desc, runs = SCENARIOS[sid]
        print(f"\n--- {sid}: {desc} ---")
        if runs is None:
            print("    MANUAL: see HIL/HIL_TEST_PLAN.md")
            summary.append((sid, "MANUAL"))
            continue
        ok = True
        for extra in runs:
            cmd = [sys.executable, HIL_PY, "--port", args.port, "--baud", str(args.baud),
                   "--seed", str(args.seed)] + extra
            print("    $ " + " ".join(extra))
            rc = subprocess.call(cmd)
            if rc == 2:
                print("    SETUP ERROR: stopping the suite")
                summary.append((sid, "SETUP ERROR"))
                return report(summary, 2)
            ok = ok and rc == 0
        summary.append((sid, "PASS" if ok else "FAIL"))
    return report(summary, 0)


def report(summary, rc):
    print("\n=== summary ===")
    counts = {}
    for sid, tag in summary:
        print(f"  {sid}: {tag}")
        counts[tag] = counts.get(tag, 0) + 1
    print("\n" + ", ".join(f"{n} {t.lower()}" for t, n in counts.items()))
    return 1 if counts.get("FAIL") else rc


if __name__ == "__main__":
    raise SystemExit(main())
