#!/usr/bin/env python3
"""
Strafe-efficiency root-cause diagnostic.

Sweeps ctrlrange (torque limit) and command speed to determine whether the
~55% strafe/forward ratio is caused by actuator torque saturation or by
fundamental roller contact mechanics.

For each (ctrlrange, cmd_speed) pair, runs the surround formation (robots at
0°, 90°, 180°, 270°) with constant world-frame velocity [cmd_speed, 0] for
N_STEPS.  Records the displacement ratio: strafing robots / forward robots.

If increasing ctrlrange significantly improves the ratio → torque-limited.
If the ratio is flat across ctrlrange → roller contact model is the cause.

Usage
-----
  python diagnose_strafe_efficiency.py
"""

import itertools
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from swarmlib.simulation.mecanum_env import MecanumTransportEnv

# ── Sweep parameters ──────────────────────────────────────────────────────
CTRLRANGES  = [10, 25, 50, 100, 200]   # Nm
CMD_SPEEDS  = [0.01, 0.05, 0.1, 0.25, 0.5, 1.0]  # m/s world-frame vx
N_ROBOTS    = 4
N_STEPS     = 80
DT_CONTROL  = 0.05

SURROUND = [
    (-0.8,  0.0, 0.0),
    ( 0.0, -0.8, np.pi / 2),
    ( 0.8,  0.0, np.pi),
    ( 0.0,  0.8, 3 * np.pi / 2),
]


def run_trial(cmd_speed: float, ctrlrange: float) -> dict:
    """Run one trial and return displacement data."""
    env = MecanumTransportEnv(
        n_robots=N_ROBOTS,
        formation=SURROUND,
        goal=(5.0, 0.0, 0.0),
        payload_pos=(0.0, 0.0),
        payload_size=(0.01, 0.01, 0.01),
        payload_mass=1.0,
        with_carriage=False,
        dt_control=DT_CONTROL,
    )

    # Override ctrlrange at runtime for all actuators
    n_act = env.model.nu
    for i in range(n_act):
        env.model.actuator_ctrlrange[i] = [-ctrlrange, ctrlrange]

    obs = env.reset()
    controls = np.tile([cmd_speed, 0.0], (N_ROBOTS, 1))

    traj = []
    torques = []
    for _ in range(N_STEPS):
        obs = env.step(controls)
        traj.append(obs["robots"][:, :2].copy())
        # Record raw PD torque (pre-clamp) from data.ctrl
        step_torques = []
        for ids in env._wheel_act_ids:
            step_torques.append([float(env.data.ctrl[aid]) for aid in ids])
        torques.append(step_torques)

    env.close()

    traj = np.array(traj)        # (N_STEPS, 4, 2)
    torques = np.array(torques)  # (N_STEPS, 4, 4)

    # Robots 0,2 face ±x (forward/backward drivers); robots 1,3 strafe
    dx = traj[-1, :, 0] - traj[0, :, 0]
    fwd_dx  = np.mean(np.abs(dx[[0, 2]]))
    strf_dx = np.mean(np.abs(dx[[1, 3]]))
    ratio   = strf_dx / fwd_dx if fwd_dx > 1e-9 else 0.0

    # Torque saturation fraction (what % of wheel-steps hit the limit)
    sat_frac = float((np.abs(torques) >= ctrlrange).mean())
    peak_torque = float(np.abs(torques).max())

    return {
        "cmd_speed":    cmd_speed,
        "ctrlrange":    ctrlrange,
        "fwd_dx":       float(fwd_dx),
        "strf_dx":      float(strf_dx),
        "ratio":        float(ratio),
        "sat_frac":     float(sat_frac),
        "peak_torque":  float(peak_torque),
    }


def main():
    figures_dir = Path(__file__).parent / "figures"
    figures_dir.mkdir(exist_ok=True)

    combos = list(itertools.product(CMD_SPEEDS, CTRLRANGES))
    print(f"Running {len(combos)} trials "
          f"({len(CMD_SPEEDS)} speeds × {len(CTRLRANGES)} ctrlranges) ...")

    results = []
    for cmd, cr in combos:
        r = run_trial(cmd, cr)
        results.append(r)
        print(f"  cmd={cmd:6.3f} m/s  ctrlrange={cr:4.0f} Nm  "
              f"fwd={r['fwd_dx']:.4f}  strf={r['strf_dx']:.4f}  "
              f"ratio={r['ratio']:.4f}  sat={r['sat_frac']*100:.1f}%")

    # ── Summary table ──────────────────────────────────────────────────
    print(f"\n{'='*80}")
    print(f"Strafe/Forward ratio matrix  (rows=cmd_speed, cols=ctrlrange)")
    print(f"{'='*80}")
    header = f"{'cmd(m/s)':>10}" + "".join(f"  {cr:>6.0f}Nm" for cr in CTRLRANGES)
    print(header)
    print("-" * len(header))

    for cmd in CMD_SPEEDS:
        row = [r for r in results if r["cmd_speed"] == cmd]
        row.sort(key=lambda x: x["ctrlrange"])
        line = f"{cmd:>10.3f}"
        for r in row:
            line += f"  {r['ratio']:>7.4f}"
        print(line)

    # ── Plot 1: ratio vs ctrlrange, one line per cmd_speed ─────────────
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    for cmd in CMD_SPEEDS:
        row = sorted([r for r in results if r["cmd_speed"] == cmd],
                     key=lambda x: x["ctrlrange"])
        crs = [r["ctrlrange"] for r in row]
        rats = [r["ratio"] for r in row]
        ax.plot(crs, rats, "o-", label=f"cmd={cmd} m/s")
    ax.set_xlabel("ctrlrange (Nm)")
    ax.set_ylabel("Strafe / Forward displacement ratio")
    ax.set_title("Effect of torque limit on strafe efficiency")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_ylim(0, 1.1)
    ax.axhline(1.0, color="k", ls="--", alpha=0.3, label="ideal (1.0)")

    # ── Plot 2: saturation fraction vs ctrlrange ──────────────────────
    ax = axes[1]
    for cmd in CMD_SPEEDS:
        row = sorted([r for r in results if r["cmd_speed"] == cmd],
                     key=lambda x: x["ctrlrange"])
        crs = [r["ctrlrange"] for r in row]
        sats = [r["sat_frac"] * 100 for r in row]
        ax.plot(crs, sats, "o-", label=f"cmd={cmd} m/s")
    ax.set_xlabel("ctrlrange (Nm)")
    ax.set_ylabel("Torque saturation (%)")
    ax.set_title("PD torque saturation vs limit")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.suptitle("Strafe Efficiency Root-Cause Diagnostic", fontsize=12)
    fig.tight_layout()
    out = figures_dir / "diagnose_strafe_efficiency.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"\nSaved {out}")


if __name__ == "__main__":
    main()
