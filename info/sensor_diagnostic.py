#!/usr/bin/env python3
"""
Sensor diagnostic — statistics + plot for base_forces and wall_forces.
Forces read via data.cfrc_ext; no taring required.

Run from repo root:
    python info/sensor_diagnostic.py [--save sensor_diag.png]
    python info/sensor_diagnostic.py --payload-mass 5.0
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt

from swarmlib.simulation.mecanum_env import MecanumTransportEnv
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation

parser = argparse.ArgumentParser()
parser.add_argument("--save",         default=None)
parser.add_argument("--payload-mass", type=float, default=2.0)
args = parser.parse_args()

N            = 4
PAYLOAD_MASS = args.payload_mass
HX, HY, HZ  = 0.60, 0.60, 0.12
DT           = 0.05
SETTLE_STEPS = 40
DRIVE_STEPS  = 60
SPEED        = 0.15

formation = face_contact_formation(N, payload_hx=HX, payload_hy=HY)
env = MecanumTransportEnv(
    n_robots=N, formation=formation, goal=(5.0, 0.0, 0.0),
    payload_pos=(0.0, 0.0), payload_mass=PAYLOAD_MASS,
    payload_size=(HX, HY, HZ), with_carriage=True, dt_control=DT,
)
env.reset()

G           = 9.81
expected_N  = PAYLOAD_MASS * G
print(f"Payload mass={PAYLOAD_MASS} kg  expected weight={expected_N:.2f} N")
print(f"Expected per-robot base force ≈ {expected_N/N:.2f} N  (no taring needed)\n")

total    = SETTLE_STEPS + DRIVE_STEPS
times    = np.zeros(total)
base_log = np.zeros((total, N))
wall_log = np.zeros((total, N))

for step in range(total):
    speed = 0.0 if step < SETTLE_STEPS else SPEED
    obs = env.step(np.tile([speed, 0.0], (N, 1)))
    times[step]    = step * DT
    base_log[step] = obs['base_forces']
    wall_log[step] = obs['wall_forces']

env.close()

sum_base  = base_log.sum(axis=1)   # total vertical contact force (N)
mass_est  = sum_base / G           # estimated payload mass (kg)

# ── Statistics ────────────────────────────────────────────────────────────────
def stats(arr, label):
    print(f"  {label:<38}  mean={arr.mean():+9.3f}  std={arr.std():8.3f}  "
          f"min={arr.min():+9.3f}  max={arr.max():+9.3f}")

for phase, sl in [("SETTLE", slice(0, SETTLE_STEPS)), ("DRIVE", slice(SETTLE_STEPS, total))]:
    print(f"── {phase} ──────────────────────────────────────────────────────")
    print("  BASE FORCES (Fz from cfrc_ext, N)")
    for i in range(N): stats(base_log[sl, i],  f"  base[{i}]")
    stats(sum_base[sl],                         "  sum(base)")
    stats(mass_est[sl],                         "  mass_est = sum(base)/g  (kg)")
    print(f"  {'expected mass':<38}  {PAYLOAD_MASS:.3f} kg")
    print("  WALL FORCES (horizontal magnitude, N)")
    for i in range(N): stats(wall_log[sl, i],  f"  wall[{i}]")
    stats(wall_log[sl].sum(axis=1),            "  sum(wall)")
    print()

# ── Plot ──────────────────────────────────────────────────────────────────────
settle_t = SETTLE_STEPS * DT
fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

ax = axes[0]
for i in range(N): ax.plot(times, base_log[:, i], label=f"robot {i}")
ax.axhline(expected_N / N, color='tab:red', ls='--', lw=1.2,
           label=f'expected {expected_N/N:.1f} N/robot')
ax.axvline(settle_t, color='k', ls='--', lw=0.8, label='drive start')
ax.set_ylabel("base Fz (N)"); ax.set_title("Per-robot base_forces (cfrc_ext, no tare)")
ax.legend(fontsize=8); ax.grid(alpha=0.3)

ax = axes[1]
ax.plot(times, mass_est, color='tab:orange', label='mass_est')
ax.axhline(PAYLOAD_MASS, color='tab:red', ls='--', lw=1.2, label=f'true {PAYLOAD_MASS} kg')
ax.axvline(settle_t, color='k', ls='--', lw=0.8)
ax.set_ylabel("mass est (kg)"); ax.set_title("Mass estimate = sum(base_forces) / g")
ax.legend(fontsize=8); ax.grid(alpha=0.3)

ax = axes[2]
for i in range(N): ax.plot(times, wall_log[:, i], label=f"robot {i}")
ax.axvline(settle_t, color='k', ls='--', lw=0.8, label='drive start')
ax.set_ylabel("wall |F| (N)"); ax.set_xlabel("Time (s)")
ax.set_title("Per-robot wall_forces (horizontal magnitude)")
ax.legend(fontsize=8); ax.grid(alpha=0.3)

fig.tight_layout()
if args.save:
    fig.savefig(args.save, dpi=150)
    print(f"Saved to {args.save}")
else:
    plt.show()
