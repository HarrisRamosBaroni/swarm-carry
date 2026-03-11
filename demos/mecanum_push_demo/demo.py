#!/usr/bin/env python3
"""
Mecanum push demo — Summit XL Steel robots, no ROS2 required.

N robots side-push a lightweight payload from x=0 to goal_x in a straight
line.  This is the same scenario as experiments/mpc_scaling/ (simple pushing,
no obstacles, no force sensing) but with real Summit XL Steel mecanum-wheel
robots instead of TurtleBot3 diff-drive.

The payload mass is derived from a low density (default 50 kg/m³) so friction
and contact forces stay manageable and the robots can push it easily.

Usage
-----
    cd demos/mecanum_push_demo
    python demo.py                          # 4 robots, default goal
    python demo.py --n-robots 2
    python demo.py --n-robots 4 --speed 0.4 --goal-x 3.0
    python demo.py --n-robots 6 --duration 60

Controls: drag to rotate, scroll to zoom, double-click to pause.
"""

import argparse
import math

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    raise SystemExit("MuJoCo not found. Install with: pip install mujoco")

from swarmlib.simulation.mecanum_env import MecanumTransportEnv

_PAYLOAD_DENSITY = 50.0   # kg/m³ — intentionally light; increase once physics is tuned


def main():
    parser = argparse.ArgumentParser(description="Mecanum push demo")
    parser.add_argument("--n-robots", type=int, default=4)
    parser.add_argument("--speed",    type=float, default=0.3,
                        help="Forward speed in m/s (world +x)")
    parser.add_argument("--goal-x",   type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=30.0, help="seconds")
    args = parser.parse_args()

    n = args.n_robots

    env = MecanumTransportEnv(
        n_robots=n,
        goal=(args.goal_x, 0.0, 0.0),
        payload_density=_PAYLOAD_DENSITY,
        with_carriage=False,
    )

    print(f"Mecanum push demo — {n} robot(s), side-push formation")
    print(f"  speed={args.speed} m/s  goal_x={args.goal_x} m")
    print(f"  payload density={_PAYLOAD_DENSITY} kg/m³")
    print("Controls: drag to rotate view, scroll to zoom, double-click to pause\n")

    env.reset()

    settle_steps = int(1.0 / env._dt)   # ~1 s for robots to settle before driving
    step = 0
    last_report = -1.0

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.azimuth   = 150
        viewer.cam.elevation = -20
        viewer.cam.distance  = max(4.0, args.goal_x * 0.6)

        while viewer.is_running() and env.time < args.duration:
            t = env.time

            if step < settle_steps:
                controls = np.zeros((n, 2))
            else:
                controls = np.tile([args.speed, 0.0], (n, 1))

            obs = env.step(controls)

            if t - last_report >= 0.5:
                payload = obs['payload']
                print(
                    f"t={t:5.1f}s  payload=({payload[0]:+.2f}, {payload[1]:+.2f}) m"
                    f"  theta={math.degrees(payload[2]):+.1f}°"
                    f"  vx={payload[3]:+.2f} m/s"
                )
                last_report = t

                dist = np.linalg.norm(payload[:2] - env.goal[:2])
                if step > settle_steps and dist < 0.10:
                    print("\nGoal reached!")
                    break

            viewer.sync()
            step += 1

    env.close()


if __name__ == "__main__":
    main()
