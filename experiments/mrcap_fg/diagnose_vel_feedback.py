#!/usr/bin/env python3
"""
Test per-robot velocity feedback loop.

Runs the surround formation diagnostic (constant world-frame command [vx, 0])
with and without velocity feedback, comparing the strafe/forward ratio.

If feedback works, the ratio should approach 1.0.

Usage
-----
  python diagnose_vel_feedback.py
"""

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from swarmlib.simulation.mecanum_env import MecanumTransportEnv

N_ROBOTS   = 4
N_STEPS    = 200       # longer run to let integral accumulate
DT_CONTROL = 0.05
CMD_SPEEDS = [0.05, 0.1, 0.25, 0.5]

SURROUND = [
    (-0.8,  0.0, 0.0),
    ( 0.0, -0.8, np.pi / 2),
    ( 0.8,  0.0, np.pi),
    ( 0.0,  0.8, 3 * np.pi / 2),
]


def run_trial(cmd_speed, vel_feedback, label):
    env = MecanumTransportEnv(
        n_robots=N_ROBOTS,
        formation=SURROUND,
        goal=(5.0, 0.0, 0.0),
        payload_pos=(0.0, 0.0),
        payload_size=(0.01, 0.01, 0.01),
        payload_mass=1.0,
        with_carriage=False,
        dt_control=DT_CONTROL,
        vel_feedback=vel_feedback,
    )
    obs = env.reset()
    controls = np.tile([cmd_speed, 0.0], (N_ROBOTS, 1))

    traj = []
    vel_history = []
    for _ in range(N_STEPS):
        obs = env.step(controls)
        traj.append(obs["robots"][:, :2].copy())
        vel_history.append(obs["robots"][:, 2:4].copy())

    env.close()
    traj = np.array(traj)             # (N_STEPS, 4, 2)
    vel_history = np.array(vel_history)  # (N_STEPS, 4, 2)

    dx = traj[-1, :, 0] - traj[0, :, 0]
    fwd_dx  = np.mean(np.abs(dx[[0, 2]]))
    strf_dx = np.mean(np.abs(dx[[1, 3]]))
    ratio   = strf_dx / fwd_dx if fwd_dx > 1e-9 else 0.0

    return {
        "label": label,
        "cmd_speed": cmd_speed,
        "fwd_dx": fwd_dx,
        "strf_dx": strf_dx,
        "ratio": ratio,
        "traj": traj,
        "vel_history": vel_history,
        "dx": dx,
    }


def main():
    figures_dir = Path(__file__).parent / "figures"
    figures_dir.mkdir(exist_ok=True)

    print(f"Running velocity feedback diagnostic ({len(CMD_SPEEDS)} speeds × 2 modes) ...\n")

    all_results = []
    for cmd in CMD_SPEEDS:
        r_off = run_trial(cmd, False, f"cmd={cmd} NO FB")
        r_on  = run_trial(cmd, True,  f"cmd={cmd} WITH FB")
        all_results.append((r_off, r_on))
        print(f"  cmd={cmd:.3f} m/s  |  NO FB: ratio={r_off['ratio']:.4f} "
              f"(fwd={r_off['fwd_dx']:.3f}, strf={r_off['strf_dx']:.3f})  |  "
              f"WITH FB: ratio={r_on['ratio']:.4f} "
              f"(fwd={r_on['fwd_dx']:.3f}, strf={r_on['strf_dx']:.3f})")

    # ── Plot: ratio comparison ─────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    cmds = [c for c in CMD_SPEEDS]
    ratios_off = [r[0]["ratio"] for r in all_results]
    ratios_on  = [r[1]["ratio"] for r in all_results]
    x = np.arange(len(cmds))
    w = 0.35
    ax.bar(x - w/2, ratios_off, w, label="No feedback", color="#F44336", alpha=0.8)
    ax.bar(x + w/2, ratios_on,  w, label="With feedback", color="#4CAF50", alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{c}" for c in cmds])
    ax.set_xlabel("Command speed (m/s)")
    ax.set_ylabel("Strafe / Forward ratio")
    ax.set_title("Strafe efficiency: feedback off vs on")
    ax.axhline(1.0, color="k", ls="--", alpha=0.3)
    ax.set_ylim(0, 1.3)
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    # ── Plot: X-velocity over time for one speed (0.25 m/s) ───────────
    ax = axes[1]
    idx = CMD_SPEEDS.index(0.25) if 0.25 in CMD_SPEEDS else 1
    r_off, r_on = all_results[idx]
    steps = np.arange(N_STEPS)
    cmd = r_off["cmd_speed"]
    # Robot 0 = forward, Robot 1 = strafe
    ax.plot(steps, r_off["vel_history"][:, 0, 0], color="#2196F3", ls="--",
            label="R0 (fwd) no FB", alpha=0.6)
    ax.plot(steps, r_off["vel_history"][:, 1, 0], color="#F44336", ls="--",
            label="R1 (strafe) no FB", alpha=0.6)
    ax.plot(steps, r_on["vel_history"][:, 0, 0], color="#2196F3",
            label="R0 (fwd) with FB")
    ax.plot(steps, r_on["vel_history"][:, 1, 0], color="#F44336",
            label="R1 (strafe) with FB")
    ax.axhline(cmd, color="k", ls=":", alpha=0.3, label=f"target {cmd} m/s")
    ax.set_xlabel("Step")
    ax.set_ylabel("World-frame vx (m/s)")
    ax.set_title(f"Velocity tracking @ cmd={cmd} m/s")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    fig.suptitle("Per-Robot Velocity Feedback Diagnostic", fontsize=12)
    fig.tight_layout()
    out = figures_dir / "diagnose_vel_feedback.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"\nSaved {out}")


if __name__ == "__main__":
    main()
