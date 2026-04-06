#!/usr/bin/env python3
"""
Research scenario demo — force sensor validation.

Demonstrates the face-contact formation with Summit XL Steel mecanum robots
and L-shaped forklift holders gripping a box payload.

  Phase 1 (0–2 s)  — payload settles onto fork bases under gravity
  Phase 2 (2 s+)   — all robots drive forward at constant speed

At the end, a matplotlib figure shows base (vertical) and wall (horizontal)
force readings over time for each robot.

Usage
-----
    cd demos/research_scenario_demo
    python demo.py                          # 4 robots, viewer + plot
    python demo.py --n-robots 2
    python demo.py --no-viewer              # headless, plot only
    python demo.py --n-robots 3 --save-plot forces.png
"""

import argparse
import time

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    raise SystemExit("MuJoCo not found. Install with: pip install mujoco")

from swarmlib.simulation.mecanum_env import MecanumTransportEnv
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation

# Payload box half-sizes (m)
PAYLOAD_HX = 0.30
PAYLOAD_HY = 0.30
PAYLOAD_HZ = 0.12


def run(n_robots, speed, duration, payload_mass, visualise):
    payload_size = (PAYLOAD_HX, PAYLOAD_HY, PAYLOAD_HZ)
    formation = face_contact_formation(n_robots,
                                       payload_hx=PAYLOAD_HX,
                                       payload_hy=PAYLOAD_HY)

    env = MecanumTransportEnv(
        n_robots=n_robots,
        formation=formation,
        goal=(5.0, 0.0, 0.0),
        payload_pos=(0.0, 0.0),
        payload_mass=payload_mass,
        payload_size=payload_size,
        with_carriage=True,
        dt_control=0.05,
    )
    obs = env.reset()

    settle_time = 2.0
    settle_steps = int(settle_time / env._dt)

    # Logging
    times = []
    base_log = []   # (steps, n, 3)
    wall_log = []   # (steps, n, 3)

    viewer = None
    if visualise:
        viewer = mujoco.viewer.launch_passive(env.model, env.data)
        viewer.cam.azimuth = 150
        viewer.cam.elevation = -25
        viewer.cam.distance = 4.0
        input("Viewer open — adjust camera, then press Enter to start...")

    step = 0
    last_report = -1.0

    while env.time < duration:
        if viewer is not None and not viewer.is_running():
            break

        t = env.time

        if step < settle_steps:
            controls = np.zeros((n_robots, 2))
        else:
            controls = np.tile([speed, 0.0], (n_robots, 1))

        obs = env.step(controls)

        times.append(t)
        base_log.append(obs['base_forces'].copy())
        wall_log.append(obs['wall_forces'].copy())

        if t - last_report >= 0.5:
            payload = obs['payload']
            base_fz = obs['base_forces'][:, 2]
            wall_fx = obs['wall_forces'][:, 0]
            phase = "settle" if step < settle_steps else "drive "
            print(
                f"[{phase}] t={t:5.1f}s  "
                f"payload=({payload[0]:+.2f}, {payload[1]:+.2f})m  "
                f"base_fz=[{', '.join(f'{f:+.1f}' for f in base_fz)}] N  "
                f"wall_fx=[{', '.join(f'{f:+.1f}' for f in wall_fx)}] N"
            )
            last_report = t

        if viewer is not None:
            viewer.sync()

        step += 1

    if viewer is not None:
        viewer.close()
    env.close()

    return np.array(times), np.array(base_log), np.array(wall_log), settle_time


def plot_forces(times, base_log, wall_log, settle_time, n_robots, save_path):
    import matplotlib.pyplot as plt

    face_labels = ["-x", "+y", "+x", "-y"]

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # Base forces — vertical component (z)
    ax = axes[0]
    for i in range(n_robots):
        label = f"Robot {i} ({face_labels[round(i * 4 / n_robots) % 4]} face)"
        ax.plot(times, base_log[:, i, 2], label=label)
    ax.axvline(settle_time, color='grey', ls='--', lw=0.8, label='drive start')
    ax.set_ylabel("Base force  Fz (N)")
    ax.set_title("Vertical load on fork bases (payload weight)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Wall forces — horizontal component (x in wall site frame)
    ax = axes[1]
    for i in range(n_robots):
        label = f"Robot {i} ({face_labels[round(i * 4 / n_robots) % 4]} face)"
        ax.plot(times, wall_log[:, i, 0], label=label)
    ax.axvline(settle_time, color='grey', ls='--', lw=0.8, label='drive start')
    ax.set_ylabel("Wall force  Fx (N)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Horizontal force on fork walls (contact / push)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Plot saved to {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Research scenario demo — force sensor validation")
    parser.add_argument("--n-robots", type=int, default=4)
    parser.add_argument("--speed", type=float, default=0.15,
                        help="Forward speed m/s during drive phase (default: 0.15)")
    parser.add_argument("--payload-mass", type=float, default=2.0, help="kg")
    parser.add_argument("--duration", type=float, default=15.0, help="seconds")
    parser.add_argument("--no-viewer", action="store_true",
                        help="Run headless (no MuJoCo viewer)")
    parser.add_argument("--save-plot", type=str, default=None,
                        help="Save plot to file instead of showing interactively")
    args = parser.parse_args()

    n = args.n_robots
    print(f"Research scenario demo — {n} robots, face-contact formation")
    print(f"  speed={args.speed} m/s  payload={args.payload_mass} kg  "
          f"box=({2*PAYLOAD_HX:.1f} x {2*PAYLOAD_HY:.1f} x {2*PAYLOAD_HZ:.1f}) m")
    print(f"  settle=2.0 s  total={args.duration} s\n")

    times, base_log, wall_log, settle_time = run(
        n_robots=n,
        speed=args.speed,
        duration=args.duration,
        payload_mass=args.payload_mass,
        visualise=not args.no_viewer,
    )

    plot_forces(times, base_log, wall_log, settle_time, n, args.save_plot)


if __name__ == "__main__":
    main()
