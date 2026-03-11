#!/usr/bin/env python3
"""
Holonomic carriage demo (no ROS2 required).

Shows holonomic robots with L-shaped forklift carriages transporting a payload:
  Phase 1 (0–1 s)  — payload settles onto the fork bases under gravity
  Phase 2 (1 s+)   — all robots drive forward at constant speed, pushing payload

Force readings are printed every 0.5 s:
  base_fz — normal load on horizontal fork base (payload weight)
  wall_fx — shear load on vertical fork wall   (reaction force while pushing)

Usage
-----
    cd demos/holonomic_carriage_demo
    python demo.py                        # 4 robots, default goal
    python demo.py --n-robots 2 --speed 0.5
    python demo.py --n-robots 4 --surrounding  # robots on all four sides
"""

import argparse
import math
import time

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    raise SystemExit("MuJoCo not found. Install with: pip install mujoco")

from swarmlib.simulation.holonomic_env import HolonomicTransportEnv
from swarmlib.simulation.generate_holonomic_scene import side_push_formation


def surrounding_formation(n: int, standoff: float = 0.35) -> list:
    """
    Place robots symmetrically around the payload.
    Each robot's +x axis points toward the payload centre (inward).
    """
    formation = []
    for i in range(n):
        angle = 2 * math.pi * i / n          # evenly spaced angles
        x = -standoff * math.cos(angle)      # offset from payload centre
        y = -standoff * math.sin(angle)
        yaw = angle + math.pi                # face inward
        formation.append((x, y, yaw))
    return formation


def main():
    parser = argparse.ArgumentParser(description="Holonomic carriage demo")
    parser.add_argument("--n-robots", type=int, default=4)
    parser.add_argument("--speed", type=float, default=0.3,
                        help="Forward speed in m/s (world +x for side-push)")
    parser.add_argument("--goal-x", type=float, default=5.0)
    parser.add_argument("--payload-mass", type=float, default=10.0, help="kg")
    parser.add_argument("--surrounding", action="store_true",
                        help="Place robots on all sides instead of side-push layout")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds")
    args = parser.parse_args()

    n = args.n_robots
    formation = surrounding_formation(n) if args.surrounding else side_push_formation(n)

    print(f"Holonomic carriage demo — {n} robot(s), "
          f"{'surrounding' if args.surrounding else 'side-push'} formation")
    print(f"  speed={args.speed} m/s  goal_x={args.goal_x} m  "
          f"payload={args.payload_mass} kg")
    print("Controls: drag to rotate view, scroll to zoom, double-click to pause\n")

    env = HolonomicTransportEnv(
        n_robots=n,
        formation=formation,
        goal=(args.goal_x, 0.0, 0.0),
        payload_mass=args.payload_mass,
    )
    env.reset()

    # Settling period: how many control steps before we drive forward
    settle_steps = int(1.0 / env._dt)

    step = 0
    last_report = -1.0

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.azimuth = 150
        viewer.cam.elevation = -20
        viewer.cam.distance = max(4.0, args.goal_x * 0.5)

        while viewer.is_running() and env.time < args.duration:
            t = env.time

            # Phase 1: hold still so payload settles onto forks
            # Phase 2: drive all robots forward in world +x
            if step < settle_steps:
                controls = np.zeros((n, 2))
            else:
                controls = np.tile([args.speed, 0.0], (n, 1))

            obs = env.step(controls)

            # Periodic console readout
            if t - last_report >= 0.5:
                payload = obs['payload']
                base_fz = obs['base_forces'][:, 2]
                wall_fx = obs['wall_forces'][:, 0]
                print(
                    f"t={t:5.1f}s  payload=({payload[0]:+.2f},{payload[1]:+.2f})m  "
                    f"base_fz={base_fz.mean():+6.1f}N  wall_fx={wall_fx.mean():+6.1f}N"
                )
                last_report = t

                # Check goal reached
                dist = np.linalg.norm(payload[:2] - env.goal[:2])
                if step > settle_steps and dist < 0.10:
                    print("\nGoal reached!")
                    break

            viewer.sync()
            step += 1

    env.close()


if __name__ == "__main__":
    main()
