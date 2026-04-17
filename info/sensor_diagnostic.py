#!/usr/bin/env python3
"""
Sensor diagnostic — statistics + plot for base_forces and wall_forces.
Run from repo root:
    python info/sensor_diagnostic.py [--save sensor_diag.png]
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt

from swarmlib.simulation.mecanum_env import MecanumTransportEnv
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation

N            = 4
PAYLOAD_MASS = 2.0
HX, HY, HZ  = 0.30, 0.30, 0.12
DT           = 0.05
SETTLE_STEPS = 40
DRIVE_STEPS  = 60
SPEED        = 0.15

parser = argparse.ArgumentParser()
parser.add_argument("--save", default=None)
args = parser.parse_args()

formation = face_contact_formation(N, payload_hx=HX, payload_hy=HY)
env = MecanumTransportEnv(
    n_robots=N, formation=formation, goal=(5.0, 0.0, 0.0),
    payload_pos=(0.0, 0.0), payload_mass=PAYLOAD_MASS,
    payload_size=(HX, HY, HZ), with_carriage=True, dt_control=DT,
)
env.reset()

G = 9.81
tare = np.array([float(env.model.body(f'robot_{i}_fork_base').mass) * G for i in range(N)])
expected_N = PAYLOAD_MASS * G

print(f"fork_base masses: {tare/G} kg   tares: {tare} N   total tare: {tare.sum():.3f} N")
print(f"Payload mass={PAYLOAD_MASS} kg  expected weight={expected_N:.2f} N\n")

total = SETTLE_STEPS + DRIVE_STEPS
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

base_tared  = base_log - tare           # (total, N)  per-robot tared Fz
sum_raw     = base_log.sum(axis=1)      # (total,)
sum_tared   = base_tared.sum(axis=1)    # (total,)
mass_est    = sum_tared / G             # (total,)

# ── Statistics ────────────────────────────────────────────────────────────────
def stats(arr, label):
    print(f"  {label:<38}  mean={arr.mean():+9.3f}  std={arr.std():8.3f}  "
          f"min={arr.min():+9.3f}  max={arr.max():+9.3f}")

for phase, sl in [("SETTLE", slice(0, SETTLE_STEPS)), ("DRIVE", slice(SETTLE_STEPS, total))]:
    print(f"── {phase} ──────────────────────────────────────────────────────")
    print("  BASE FORCES (raw Fz, N)")
    for i in range(N): stats(base_log[sl, i],    f"  base_raw[{i}]")
    stats(sum_raw[sl],                            "  sum(base_raw)")
    print("  BASE FORCES (tared Fz, N)")
    for i in range(N): stats(base_tared[sl, i],  f"  base_tared[{i}]")
    stats(sum_tared[sl],                          "  sum(base_tared)")
    stats(mass_est[sl],                           "  mass_est = sum_tared/g  (kg)")
    print(f"  {'expected mass':<38}  {PAYLOAD_MASS:.3f} kg")
    print("  WALL FORCES (Fx, N)")
    for i in range(N): stats(wall_log[sl, i],    f"  wall_fx[{i}]")
    stats(wall_log[sl].sum(axis=1),              "  sum(wall_fx)")
    print()

# ── Plot ──────────────────────────────────────────────────────────────────────
settle_t = SETTLE_STEPS * DT
fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

ax = axes[0]
for i in range(N): ax.plot(times, base_log[:, i], label=f"robot {i} raw")
ax.axvline(settle_t, color='k', ls='--', lw=0.8, label='drive start')
ax.set_ylabel("base Fz raw (N)"); ax.set_title("Per-robot base_forces raw — large constraint forces cancel in sum")
ax.legend(fontsize=8); ax.grid(alpha=0.3)

ax = axes[1]
for i in range(N): ax.plot(times, base_tared[:, i], label=f"robot {i} tared")
ax.axhline(0, color='grey', ls=':', lw=0.7)
ax.axvline(settle_t, color='k', ls='--', lw=0.8)
ax.plot(times, sum_tared, color='k', lw=1.5, label='sum (tared)')
ax.axhline(expected_N, color='tab:red', ls='--', lw=1.2, label=f'expected {expected_N:.1f} N')
ax.set_ylabel("base Fz tared (N)"); ax.set_title("Tared base_forces per robot + sum vs expected weight")
ax.legend(fontsize=8); ax.grid(alpha=0.3)

ax = axes[2]
ax.plot(times, mass_est, color='tab:orange', label='mass_est')
ax.axhline(PAYLOAD_MASS, color='tab:red', ls='--', lw=1.2, label=f'true {PAYLOAD_MASS} kg')
ax.axvline(settle_t, color='k', ls='--', lw=0.8)
ax.set_ylabel("mass est (kg)"); ax.set_title("Mass estimate from tared sum(base_forces) / g")
ax.legend(fontsize=8); ax.grid(alpha=0.3)

ax = axes[3]
for i in range(N): ax.plot(times, wall_log[:, i], label=f"robot {i}")
ax.axvline(settle_t, color='k', ls='--', lw=0.8, label='drive start')
ax.set_ylabel("wall Fx (N)"); ax.set_xlabel("Time (s)")
ax.set_title("Per-robot wall_forces (Fx) — horizontal contact")
ax.legend(fontsize=8); ax.grid(alpha=0.3)

fig.tight_layout()
if args.save:
    fig.savefig(args.save, dpi=150)
    print(f"Saved to {args.save}")
else:
    plt.show()
