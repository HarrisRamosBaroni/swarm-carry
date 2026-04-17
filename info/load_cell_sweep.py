#!/usr/bin/env python3
"""
Load-cell parameter sweep with contact co-tuning.

Co-tuning rule (stability condition):
    k_contact_eff ≈ m_eff / tc²   where tc = solrefcontact time constant
    Stable when k_spring << k_contact_eff, i.e. tc < sqrt(m_eff / (SAFETY · k))

Sweep: for each k, compute tc_opt = sqrt(m_eff / (SAFETY · k)),
then run tc ∈ {0.5·tc_opt, tc_opt, 2·tc_opt} to bracket the stability boundary.
ζ is fixed at 1.5 (slightly overdamped) — previous sweep showed ζ has no effect
once contact is stable.

Run from repo root:
    python info/load_cell_sweep.py
"""

import argparse
import math
import tempfile
from pathlib import Path

import numpy as np

# ── CLI ───────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--n-robots",     type=int,   default=4)
parser.add_argument("--payload-mass", type=float, default=2.0)
parser.add_argument("--settle-steps", type=int,   default=80)
args = parser.parse_args()

N            = args.n_robots
M            = args.payload_mass
SETTLE_STEPS = args.settle_steps
G            = 9.81
HX, HY, HZ  = 0.60, 0.60, 0.12
ZETA         = 1.5
SAFETY       = 15.0   # k_contact_eff / k_spring target ratio

K_VALUES = [1e3, 5e3, 2e4, 8e4]

import swarmlib.simulation.generate_mecanum_scene as _gms
import swarmlib.simulation.mecanum_env as _me
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation

m_plate = _gms.LOAD_CELL_MASS
m_eff   = M / N + m_plate

print(f"Sweep: N={N}  M={M} kg  m_plate={m_plate} kg  ζ={ZETA}  safety={SAFETY:.0f}")
print(f"m_eff = {m_eff:.3f} kg   physics dt = 0.002 s   settle = {SETTLE_STEPS}×0.05 s")
print(f"Co-tuning: tc_opt = sqrt(m_eff / (safety·k))")
print()

HDR_FMT = (
    f"{'k':>8}  {'tc (ms)':>9}  {'tc_ratio':>9}  {'d':>8}  "
    f"{'x_stat mm':>10}  {'mean kg':>9}  {'std kg':>8}  {'err%':>7}  {'result':>8}"
)
print(HDR_FMT)
print("─" * len(HDR_FMT))

orig_k = _gms.LOAD_CELL_STIFFNESS
orig_d = _gms.LOAD_CELL_DAMPING

for k in K_VALUES:
    d_crit  = 2.0 * math.sqrt(k * m_eff)
    d       = ZETA * d_crit
    tc_opt  = math.sqrt(m_eff / (SAFETY * k))
    x_mm    = (M * G / N) / k * 1e3

    for tc_mult in [0.5, 1.0, 2.0]:
        tc = tc_opt * tc_mult

        _gms.LOAD_CELL_STIFFNESS = k
        _gms.LOAD_CELL_DAMPING   = d

        tmpdir = tempfile.mkdtemp(prefix=f"lc_k{k:.0f}_tc{tc*1000:.2f}_")
        try:
            formation = face_contact_formation(N, payload_hx=HX, payload_hy=HY)
            env = _me.MecanumTransportEnv(
                n_robots=N, formation=formation,
                goal=(5.0, 0.0, 0.0),
                payload_pos=(0.0, 0.0),
                payload_mass=M,
                payload_size=(HX, HY, HZ),
                with_carriage=True,
                dt_control=0.05,
                scenes_dir=tmpdir,
                contact_timeconst=tc,
            )
            env._load_k = k
            env._load_d = d
            env.reset()

            tare = np.array([
                float(env.model.body(f'robot_{i}_fork_base').mass) * G
                for i in range(N)
            ])
            mass_samples = np.zeros(SETTLE_STEPS)
            for step in range(SETTLE_STEPS):
                obs = env.step(np.zeros((N, 2)))
                mass_samples[step] = (obs['base_forces'] - tare).sum() / G
            env.close()

        except Exception as exc:
            print(f"{k:>8.0f}  {tc*1000:>9.3f}  {tc_mult:>9.1f}  {d:>8.1f}  "
                  f"{x_mm:>10.2f}  ERROR: {exc}")
            continue
        finally:
            for f in Path(tmpdir).glob("*.xml"):
                f.unlink()
            Path(tmpdir).rmdir()

        half    = SETTLE_STEPS // 2
        late    = mass_samples[half:]
        mean_kg = late.mean()
        std_kg  = late.std()
        err_pct = (mean_kg - M) / M * 100.0
        ok      = std_kg < 0.05 and abs(err_pct) < 5.0
        result  = "OK" if ok else ("osc" if std_kg >= 0.05 else "bias")

        print(
            f"{k:>8.0f}  {tc*1000:>9.3f}  {tc_mult:>9.1f}  {d:>8.1f}  "
            f"{x_mm:>10.2f}  {mean_kg:>9.3f}  {std_kg:>8.4f}  {err_pct:>+7.2f}%  {result:>8}"
        )
    print()   # blank line between k groups

_gms.LOAD_CELL_STIFFNESS = orig_k
_gms.LOAD_CELL_DAMPING   = orig_d

print("result: OK = std<0.05 kg and |err|<5%  |  osc = oscillating  |  bias = systematic error")
print(f"tc_ratio: multiplier on tc_opt = sqrt(m_eff/(safety·k))")
print(f"         < 1.0 → tighter contact (more stable)  |  > 1.0 → looser (cheaper but risky)")
