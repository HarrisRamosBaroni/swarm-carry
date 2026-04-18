# Load Cell Investigation Summary

## Goal

Implement per-robot force sensing in the MuJoCo swarm simulation so that each robot can estimate the share of the payload's weight it is bearing. This is needed for distributed mass estimation in the swarm controller.

---

## Background: The Indeterminacy Problem

The original approach used MuJoCo `<force>` sensors at sites on the fork bodies. These gave unstable, noisy readings because of **rigid-rigid static indeterminacy**: N rigid robots pressing on a rigid payload creates an overdetermined contact system; MuJoCo's LCP solver finds *some* solution but it has no physical meaning per-robot (it depends on initial conditions and solver tolerances).

The conceptually correct fix (suggested by prior LLM conversation in `info/simulate_load_cells.md`) is to **break the indeterminacy** by adding a compliant element — a spring-damper slide joint on each fork body. With a unique spring deflection per robot, the equilibrium is uniquely determined and the per-robot load is physically meaningful.

---

## Approach: Spring-Displacement Load Cell

Each `fork_base` and `fork_wall` body is attached to the robot chassis via a 1-DOF slide joint with stiffness `k` and damping `d`. Force is computed as `F = -(k·q + d·q̇)`.

### Files modified
- `swarmlib/simulation/generate_mecanum_scene.py`: adds slide joints with spring/damper to fork bodies
- `swarmlib/simulation/mecanum_env.py`: reads `data.qpos`/`data.qvel` for joint displacements, computes forces
- `info/sensor_diagnostic.py`: diagnostics script (updated several times)
- `info/load_cell_sweep.py`: parameter sweep script (written during investigation)

---

## Parameter Sweep Findings

### Low-k sweep (k ∈ [600, 700] N/m, HX=HY=0.60 m, M=2.0 kg, N=4)

| k (N/m) | ζ | mass_est (kg) | err% | verdict |
|---|---|---|---|---|
| 600 | 1.0–4.0 | 1.82 | −9% | FAIL |
| 650 | 1.0–4.0 | 2.00 | ~0% | **OK** |
| 660 | 1.0–4.0 | 2.03 | +1.4% | OK |
| 700 | 1.0–4.0 | 2.18 | +9% | FAIL |

**Key observation:** ζ (damping ratio) has essentially no effect on mean error or std. The spring settles analytically in <0.06 s (well within the 4 s settle window), so damping was never the bottleneck. **k is the only parameter that matters for accuracy.**

There is a narrow zero-error window around k ≈ 651 N/m. Error changes sign across this window (+/−9% for ±50 N/m deviation).

### High-k sweep (k ∈ [1e3, 5e3, 2e4, 8e4] N/m, co-tuned contact model)

| k (N/m) | mass_est (kg) | err% | failure mode |
|---|---|---|---|
| 1000 | 3.05 | +52% | bias |
| 5000 | 15.2 | +660% | oscillation |
| 20000 | 48.9 | +2344% | oscillation |
| 80000 | 110 | +5400% | oscillation |

**Key observation:** Contact model `solref` tuning (`tc` from 0.5× to 2× of analytically predicted value) had essentially zero effect on results. This ruled out contact-model stiffness as the root cause.

**Root cause of high-k failure:** At k ≥ ~2000 N/m, the spring rebound is strong enough to cause **contact chattering** — the payload repeatedly bounces off and re-contacts the fork plate at high frequency. This inflates the mean reading by orders of magnitude. This is not a numerical integration issue; it is a physical instability between the spring stiffness and the contact impulse magnitude.

**Root cause of low-k accuracy window:** The narrow k ≈ 651 window appears to be an accidental cancellation between the range-limit soft constraint force (which adds an unmeasured component when deflection > range) and the spring reading. This calibration is specific to this payload mass and formation geometry — not a principled design.

---

## Attempted Alternative: `data.cfrc_ext`

**Hypothesis:** MuJoCo stores computed contact forces per body in `data.cfrc_ext[body_id, 3:6]`. Reading this directly would bypass the spring-stiffness tradeoff entirely.

**Result:** All readings were zero for all configurations. 

**Finding:** `data.cfrc_ext` is an **input** field in MuJoCo — it is for *applying* external Cartesian forces to bodies (equivalent to `xfrc_applied` in a different frame). It is NOT populated by the constraint solver with contact force outputs. The correct MuJoCo fields for contact force outputs are `data.efc_force` (constraint space) and `data.contact` (individual contact structures accessed via `mujoco.mj_contactForce()`), neither of which gives a simple per-body scalar.

---

## Current State (end of investigation session)

### Parameters
```python
LOAD_CELL_STIFFNESS = 500.0   # N/m
LOAD_CELL_DAMPING   = 40.0    # N·s/m  (ζ ≈ 1.15 w.r.t. m_eff = 0.6 kg)
LOAD_CELL_RANGE     = 0.025   # m  (±25 mm)
```

Expected equilibrium deflection: `(M·g/N + tare) / k = 5.886/500 = 11.8 mm` — within ±25 mm range.

### Diagnostic output (M=1, 2, 5 kg all give same result)

```
base[0] ≈ +33.6 N,  base[1] ≈ +23.6 N,  base[2] ≈ +13.6 N,  base[3] ≈ +4.0 N
mass_est ≈ +7.6 kg  (regardless of payload mass)
```

**Critical finding:** The mass estimate is ~7.6 kg and **does not change with payload mass** (tested M=1, 2, 5 kg). This proves the current readings do not measure payload contact force at all.

**Per-robot asymmetry:** base[0] ≈ 8× base[3], decreasing nearly linearly. A symmetric N=4 formation should give equal readings per robot.

**Interpretation:** The springs are measuring something structural — likely gravitational loading from the robot's own geometry or the fork bodies pressing against the chassis or floor due to the soft spring (k=500) allowing excessive deflection. The 11.8 mm equilibrium deflection changes the fork geometry enough that the payload contacts the robots in an unintended configuration (possibly only 1–2 robots rather than all 4).

---

## Open Questions / Things to Investigate in a New Session

1. **Why are readings asymmetric?** With face_contact_formation and a symmetric square payload, all 4 robots should carry equal load. Visualise the scene to see if all 4 fork_base plates are actually in contact with the payload bottom face.

2. **Is the payload resting on the fork plates at all?** With k=500 and 11.8 mm deflection, the fork plate drops significantly. Check whether the payload is instead contacting the floor, the fork walls, or only 1–2 plates.

3. **Correct contact force reading in MuJoCo:** The right approach for per-body contact force output is to iterate over `data.contact` (shape `[ncon]`, each a `mjContact` struct) and call `mujoco.mj_contactForce(model, data, contact_id, result)`. This returns the contact force in the contact frame, which can then be rotated to world frame and accumulated per body. This completely bypasses the spring-stiffness problem and gives the constraint solver's ground truth.

4. **Does any k give correct readings for the current geometry?** The old sweep at k≈650 gave correct readings but the diagnostic at k=500 does not. The geometry (HX=HY) may matter: the sweep used HX=HY=0.60, while the diagnostic uses HX=HY=0.30. Re-run the sweep with HX=HY=0.30.

5. **Visualise before measuring.** Run the research_scenario_demo with a viewer to confirm the fork plates are actually contacting the payload bottom before trusting any force readout.

---

## Analytic Model (validated for damping, not for accuracy)

For the coupled system (N springs supporting payload M):
```
m_eff   = M/N + m_plate = 2/4 + 0.1 = 0.6 kg
ω_n     = sqrt(k / m_eff)
ζ       = d / (2·sqrt(k·m_eff))
t_settle≈ 4·m_eff / d   (2% settling, overdamped)
x_stat  = (M·g/N) / k
```

The analytic model correctly predicts that ζ has negligible effect once above critical (t_settle << settle window). It fails to predict the k-dependent bias because it does not model contact chattering or range-limit soft constraints.

---

## Code State at End of Session

The code is in a partially-reworked state. The spring displacement approach is wired up correctly (qpos/qvel read, internal tare subtraction), but k=500 produces wrong readings. Before changing approach, it is worth:
- Visualising the scene to diagnose the geometric issue
- Testing whether the old k≈651 sweep (which used HX=HY=0.60) gives correct readings with the current code as a sanity check
