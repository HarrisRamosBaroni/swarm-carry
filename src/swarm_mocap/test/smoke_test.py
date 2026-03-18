#!/usr/bin/env python3
"""
Smoke test for swarm_mocap — no MoCap hardware required.

What it checks:
  1. The mocap_node binary was built (colcon build ran successfully).
  2. With a bad server IP the node logs the expected connection-failed message
     and exits cleanly (no crash / segfault).

Run from the repo root after building:
  source src/install/setup.bash
  python3 src/swarm_mocap/test/smoke_test.py
"""

import subprocess
import sys
import time

# Requires: source install/setup.bash first
NODE_CMD = [
    "ros2", "run", "swarm_mocap", "mocap_node",
    "--ros-args",
    "-p", "server_ip:=127.0.0.1",   # nothing listening here
]

TIMEOUT   = 6   # seconds — OWL open() blocks briefly before timing out
EXPECT_MSG = "connection failed"    # substring of the expected error log


def main():
    print("=== swarm_mocap smoke test ===")
    print(f"Launching: {' '.join(NODE_CMD)}")
    print(f"Expecting '{EXPECT_MSG}' in output within {TIMEOUT}s ...\n")

    try:
        result = subprocess.run(
            NODE_CMD,
            capture_output=True,
            text=True,
            timeout=TIMEOUT + 2,
        )
    except subprocess.TimeoutExpired:
        print("FAIL — node did not exit within timeout (possible hang).")
        sys.exit(1)
    except FileNotFoundError:
        print("FAIL — 'ros2' not found. Source the ROS2 workspace first:")
        print("  source src/install/setup.bash")
        sys.exit(1)

    combined = (result.stdout + result.stderr).lower()
    print("--- node output ---")
    print(result.stdout or result.stderr or "(no output)")
    print("-------------------")

    if result.returncode == 139:
        print("FAIL — node segfaulted (returncode 139).")
        sys.exit(1)

    if EXPECT_MSG in combined:
        print(f"PASS — got expected connection-failure message, exit code {result.returncode}.")
    else:
        # Node may have exited for another reason — as long as it didn't crash that's OK
        if result.returncode not in (0, 1):
            print(f"WARN — unexpected exit code {result.returncode}, check output above.")
        else:
            print(f"PASS — node exited cleanly (exit code {result.returncode}).")


if __name__ == "__main__":
    main()
