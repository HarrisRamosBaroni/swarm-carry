#!/usr/bin/env python3
"""
Contact diagnostic: answers three questions before any fix is attempted.

  Q1. Are fork_base geoms in contact with the payload?
  Q2. How far are the fork_base_slide joints deflecting?
  Q3. Are range limits being exceeded?

Run from repo root:
    python info/contact_diagnostic.py [--payload-mass 2.0]
"""
import argparse
import numpy as np
import mujoco

from swarmlib.simulation.mecanum_env import MecanumTransportEnv
from swarmlib.simulation.generate_mecanum_scene import (
    face_contact_formation, LOAD_CELL_STIFFNESS, LOAD_CELL_DAMPING, LOAD_CELL_RANGE
)

parser = argparse.ArgumentParser()
parser.add_argument("--payload-mass", type=float, default=2.0)
parser.add_argument("--settle-steps", type=int, default=40)
args = parser.parse_args()

N, HX, HY, HZ = 4, 0.60, 0.60, 0.12
M = args.payload_mass
G = 9.81

formation = face_contact_formation(N, payload_hx=HX, payload_hy=HY)
env = MecanumTransportEnv(
    n_robots=N, formation=formation, goal=(5.0, 0.0, 0.0),
    payload_pos=(0.0, 0.0), payload_mass=M,
    payload_size=(HX, HY, HZ), with_carriage=True, dt_control=0.05,
)
env.reset()

model, data = env.model, env.data

# ── Build lookup tables ────────────────────────────────────────────────────────
payload_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
payload_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "payload_geom")

fork_base_body_ids = []
fork_base_geom_ids = []
for i in range(N):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"robot_{i}_fork_base")
    fork_base_body_ids.append(bid)
    # geoms: iterate model.geom_bodyid to find geoms belonging to this body
    gids = [g for g in range(model.ngeom) if model.geom_bodyid[g] == bid]
    fork_base_geom_ids.append(gids)

print(f"Payload mass={M} kg  k={LOAD_CELL_STIFFNESS} N/m  d={LOAD_CELL_DAMPING} N·s/m  range=±{LOAD_CELL_RANGE*1000:.0f}mm")
print(f"Expected per-robot deflection at equilibrium: {(M*G/N + 0.1*G)/LOAD_CELL_STIFFNESS*1000:.2f} mm")
print(f"Range limit: ±{LOAD_CELL_RANGE*1000:.0f} mm → limit hit if |q| > {LOAD_CELL_RANGE*1000:.0f} mm\n")

# ── Settle ─────────────────────────────────────────────────────────────────────
for _ in range(args.settle_steps):
    env.step(np.zeros((N, 2)))

# ── Q1: Contact pairs ─────────────────────────────────────────────────────────
print(f"── Q1: Contacts (ncon={data.ncon}) ──────────────────────────────────────")
fork_payload_contacts = {i: [] for i in range(N)}
for c_id in range(data.ncon):
    c = data.contact[c_id]
    g1, g2 = int(c.geom1), int(c.geom2)
    b1 = int(model.geom_bodyid[g1])
    b2 = int(model.geom_bodyid[g2])
    n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or f"geom{g1}"
    n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or f"geom{g2}"
    # Check if this contact involves payload and a fork_base
    involves_payload = (b1 == payload_body_id or b2 == payload_body_id)
    robot_idx = None
    for i, bid in enumerate(fork_base_body_ids):
        if b1 == bid or b2 == bid:
            robot_idx = i
            break
    if involves_payload and robot_idx is not None:
        result = np.zeros(6)
        mujoco.mj_contactForce(model, data, c_id, result)
        fork_payload_contacts[robot_idx].append(result[0])
        print(f"  robot {robot_idx}: {n1} ↔ {n2}  |  normal force = {result[0]:.3f} N")

for i in range(N):
    total = sum(fork_payload_contacts[i])
    print(f"  robot {i} total fork_base↔payload normal force: {total:.3f} N")
total_contact = sum(sum(v) for v in fork_payload_contacts.values())
print(f"  sum all = {total_contact:.3f} N  →  mass_est = {total_contact/G:.3f} kg  (true={M} kg)")

# ── Q2 & Q3: Joint deflections ────────────────────────────────────────────────
print(f"\n── Q2/Q3: Fork base slide joint state ───────────────────────────────────")
for i in range(N):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"robot_{i}_fork_base_slide")
    qpa = int(model.jnt_qposadr[jid])
    doa = int(model.jnt_dofadr[jid])
    q   = float(data.qpos[qpa])
    qd  = float(data.qvel[doa])
    f_spring = -(LOAD_CELL_STIFFNESS * q + LOAD_CELL_DAMPING * qd)
    at_limit = abs(q) > LOAD_CELL_RANGE * 0.95
    print(f"  robot {i}: q={q*1000:+7.2f} mm  qd={qd:+6.3f} m/s  "
          f"F_spring={f_spring:+7.3f} N  "
          f"{'*** RANGE LIMIT ***' if at_limit else ''}")

# ── Q2 bonus: payload z ───────────────────────────────────────────────────────
pz = float(data.xpos[payload_body_id][2])
print(f"\n  Payload CoM z = {pz:.4f} m")
print(f"  Payload bottom face z = {pz - HZ:.4f} m")
from swarmlib.simulation.generate_mecanum_scene import FORK_TOP_Z_WORLD
print(f"  Fork plate top (q=0) z = {FORK_TOP_Z_WORLD:.4f} m")
print(f"  Expected fork top at equilibrium z = {FORK_TOP_Z_WORLD - (M*G/N + 0.1*G)/LOAD_CELL_STIFFNESS:.4f} m")

# ── Current env readings ──────────────────────────────────────────────────────
obs = env.step(np.zeros((N, 2)))
print(f"\n── Current env base_forces (spring method, tare subtracted) ─────────────")
for i in range(N):
    print(f"  robot {i}: {obs['base_forces'][i]:+.3f} N")
print(f"  sum = {obs['base_forces'].sum():.3f} N  →  mass_est = {obs['base_forces'].sum()/G:.3f} kg")

env.close()
