#!/usr/bin/env python3
"""
Formation drift diagnostic — no payload interaction.

Places the payload 100 m away so robots never touch it, then commands all
robots with the same world-frame velocity [vx=0.5, vy=0] and records their
actual world-frame trajectories.

Two trials:
  1. Surround formation — robots start at evenly-spaced yaw angles (0°, 90°, 180°, 270°)
  2. Aligned formation  — all robots start facing +x (yaw=0)

If the mecanum world→body frame conversion is correct, both trials should
produce identical trajectories for all robots regardless of initial yaw.
Any deviation is purely a robot-model or controller artefact, not payload interaction.

Usage
-----
  python diagnose_formation.py            # headless, saves figures/diagnose_*.png
  python diagnose_formation.py --vis      # open MuJoCo viewer for each trial
"""

import argparse
import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from swarmlib.simulation.mecanum_env import MecanumTransportEnv

N_ROBOTS   = 4
CMD_VX     = 0.5    # m/s world-frame x command given to every robot
CMD_VY     = 0.0
N_STEPS    = 80
DT_CONTROL = 0.05


def make_env(formation, n_robots=N_ROBOTS):
    """Create env with a tiny payload (1 cm box) that can't collide with robots."""
    return MecanumTransportEnv(
        n_robots=n_robots,
        formation=formation,
        goal=(5.0, 0.0, 0.0),
        payload_pos=(0.0, 0.0),
        payload_size=(0.01, 0.01, 0.01),  # negligible — no contact with robots
        payload_mass=1.0,
        with_carriage=False,
        dt_control=DT_CONTROL,
    )


def run_trial(formation, label, visualise=False):
    """
    Run N_STEPS with constant command [CMD_VX, CMD_VY] for all robots.
    Returns robot trajectories as (n_steps, n_robots, 2) array.
    """
    env = make_env(formation)
    obs = env.reset()
    n   = env.n_robots
    controls = np.tile([CMD_VX, CMD_VY], (n, 1))

    viewer = None
    if visualise:
        import mujoco.viewer as mjv
        viewer = mjv.launch_passive(env.model, env.data)
        input(f"  [{label}] Viewer open — press Enter to start...")

    traj = []   # list of (n_robots, 2) position snapshots

    for _ in range(N_STEPS):
        if viewer is not None and not viewer.is_running():
            break
        obs = env.step(controls)
        traj.append(obs["robots"][:, :2].copy())   # (n, 2)
        if viewer is not None:
            viewer.sync()
            time.sleep(DT_CONTROL)

    if viewer is not None:
        viewer.close()

    env.close()
    return np.array(traj)   # (n_steps, n_robots, 2)


def plot_trial(traj, formation, label, out_path):
    """Plot XY trajectory per robot and X/Y displacement vs step."""
    n_robots = traj.shape[1]
    steps    = np.arange(len(traj))
    colors   = ["#2196F3", "#F44336", "#4CAF50", "#FF9800"]

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))

    # --- XY paths ---
    ax = axes[0]
    for i in range(n_robots):
        yaw_deg = np.degrees(formation[i][2])
        ax.plot(traj[:, i, 0], traj[:, i, 1],
                color=colors[i], label=f"robot {i} (yaw={yaw_deg:.0f}°)")
        ax.plot(*traj[0,  i], "o", color=colors[i], ms=5)
        ax.plot(*traj[-1, i], "s", color=colors[i], ms=5)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("XY trajectories"); ax.legend(fontsize=8)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)

    # --- X displacement over time ---
    ax = axes[1]
    for i in range(n_robots):
        dx = traj[:, i, 0] - traj[0, i, 0]
        ax.plot(steps, dx, color=colors[i], label=f"robot {i}")
    ax.set_xlabel("step"); ax.set_ylabel("Δx (m)")
    ax.set_title("X displacement (should be identical)"); ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # --- Y displacement over time ---
    ax = axes[2]
    for i in range(n_robots):
        dy = traj[:, i, 1] - traj[0, i, 1]
        ax.plot(steps, dy, color=colors[i], label=f"robot {i}")
    ax.set_xlabel("step"); ax.set_ylabel("Δy (m)")
    ax.set_title("Y displacement (should be ~0)"); ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.suptitle(f"{label}\nCommand: vx={CMD_VX}, vy={CMD_VY} (world frame, all robots)",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  saved {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Formation drift diagnostic")
    parser.add_argument("--vis", action="store_true",
                        help="Open MuJoCo viewer for each trial")
    args = parser.parse_args()

    figures_dir = Path(__file__).parent / "figures"
    figures_dir.mkdir(exist_ok=True)

    # Trial 1: surround formation — different yaw per robot
    surround = [
        (-0.8,  0.0, 0.0),          # behind payload, facing +x  (yaw=0°)
        ( 0.0, -0.8, np.pi/2),      # right of payload, facing +y (yaw=90°)
        ( 0.8,  0.0, np.pi),        # in front of payload, facing -x (yaw=180°)
        ( 0.0,  0.8, 3*np.pi/2),    # left of payload, facing -y (yaw=270°)
    ]

    # Trial 2: aligned formation — all facing +x
    aligned = [
        (-0.8,  0.0, 0.0),
        ( 0.0, -0.8, 0.0),
        ( 0.8,  0.0, 0.0),
        ( 0.0,  0.8, 0.0),
    ]

    print("\nTrial 1: surround formation (robots at 0°, 90°, 180°, 270°)")
    traj_surround = run_trial(surround, "Surround formation", visualise=args.vis)
    plot_trial(traj_surround, surround,
               "Trial 1: surround formation (different yaw per robot)",
               figures_dir / "diagnose_surround.png")

    print("\nTrial 2: aligned formation (all robots yaw=0°)")
    traj_aligned = run_trial(aligned, "Aligned formation", visualise=args.vis)
    plot_trial(traj_aligned, aligned,
               "Trial 2: aligned formation (all robots yaw=0°)",
               figures_dir / "diagnose_aligned.png")

    # Summary: spread in X displacement at final step
    print("\nSpread in final Δx (should be ~0 if conversion is correct):")
    for label, traj in [("surround", traj_surround), ("aligned", traj_aligned)]:
        dx_final = traj[-1, :, 0] - traj[0, :, 0]
        print(f"  {label}: Δx = {np.round(dx_final, 3)}  "
              f"(range {dx_final.max()-dx_final.min():.4f} m)")

    dy_s = traj_surround[-1, :, 1] - traj_surround[0, :, 1]
    dy_a = traj_aligned[-1, :, 1]  - traj_aligned[0, :, 1]
    print(f"\nSpread in final Δy (should be ~0):")
    print(f"  surround: Δy = {np.round(dy_s, 3)}")
    print(f"  aligned:  Δy = {np.round(dy_a, 3)}")


if __name__ == "__main__":
    main()
