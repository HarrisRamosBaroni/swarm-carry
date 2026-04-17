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
PAYLOAD_HX = 0.60
PAYLOAD_HY = 0.60
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

    # Tare: the fork_base plate sits on its own load-cell spring, so at rest the
    # spring reads its self-weight (≈0.1 kg × g ≈ 1 N per robot).
    G = 9.81
    tare_per_robot = np.array([
        float(env.model.body(f'robot_{i}_fork_base').mass) * G
        for i in range(n_robots)
    ])
    print(f"  fork_base mass: {tare_per_robot / G} kg  "
          f"tare: {tare_per_robot} N")

    settle_time = 2.0
    settle_steps = int(settle_time / env._dt)

    # Logging
    times = []
    base_log = []   # (steps, n)
    wall_log = []   # (steps, n)

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
            base_fz = obs['base_forces']
            wall_fx = obs['wall_forces']
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

    return np.array(times), np.array(base_log), np.array(wall_log), settle_time, tare_per_robot


def plot_forces(times, base_log, wall_log, settle_time, n_robots, payload_mass,
                tare_per_robot, save_path):
    import matplotlib.pyplot as plt

    face_labels = ["-x", "+y", "+x", "-y"]
    expected_weight = payload_mass * 9.81
    total_tare = tare_per_robot.sum()

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Base forces — tared vertical component per robot
    ax = axes[0]
    for i in range(n_robots):
        label = f"Robot {i} ({face_labels[round(i * 4 / n_robots) % 4]} face)"
        ax.plot(times, base_log[:, i] - tare_per_robot[i], label=label)
    ax.axhline(0, color='grey', ls=':', lw=0.8)
    ax.axvline(settle_time, color='grey', ls='--', lw=0.8, label='drive start')
    ax.set_ylabel("Tared base Fz (N)")
    ax.set_title("Vertical load on fork bases — tared (fork-base self-weight subtracted)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Total base Fz: raw vs tared vs expected mg
    ax = axes[1]
    total_fz = base_log.sum(axis=1)
    tared_fz  = total_fz - total_tare
    ax.plot(times, total_fz,  color='tab:blue',   label=f'Raw total Fz (includes tare {total_tare:.1f} N)')
    ax.plot(times, tared_fz,  color='tab:orange', label='Tared total Fz')
    ax.axhline(expected_weight, color='tab:red', ls='--', lw=1.2,
               label=f'Expected mg = {expected_weight:.1f} N')
    ax.axvline(settle_time, color='grey', ls='--', lw=0.8, label='drive start')
    ax.set_ylabel("Total Fz (N)")
    ax.set_title("Total vertical load: raw vs tared vs expected payload weight")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Wall forces — horizontal component (x in wall site frame)
    ax = axes[2]
    for i in range(n_robots):
        label = f"Robot {i} ({face_labels[round(i * 4 / n_robots) % 4]} face)"
        ax.plot(times, wall_log[:, i], label=label)
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
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     description="Research scenario demo — force sensor validation")
    parser.add_argument("--n-robots", type=int, default=4)
    parser.add_argument("--speed", type=float, default=0.15,
                        help="Forward speed m/s during drive phase")
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

    times, base_log, wall_log, settle_time, tare_per_robot = run(
        n_robots=n,
        speed=args.speed,
        duration=args.duration,
        payload_mass=args.payload_mass,
        visualise=not args.no_viewer,
    )

    plot_forces(times, base_log, wall_log, settle_time, n, args.payload_mass,
                tare_per_robot, args.save_plot)


if __name__ == "__main__":
    main()
