# `centralised_force_fg_cvel` — Formulation vs Implementation Review

**Formulation:** `experiments/centralised_force_fg_cvel/PROBLEM_STATEMENT.md`
**Implementation:** `swarmlib/controllers/force_centralised_controller_cvel.py`


## Force usage

- **Wall forces** (`wall_forces[i]`): projected from robot-local to world frame, summed into `F_world`, then baked as a fixed constant into every horizon step of the centroid motion model factor `(C_j, V_j, C_{j+1}, M)`. Also used post-solve for velocity update: `V += F_world / M * dt`.
- **Base forces** (`base_forces[i]`): summed and divided by `g = 9.81` to produce a mass prior on the `M` node.
- Forces are **not** decision variables — the FG never optimises over them.
- The same force snapshot is stamped across all `N` horizon steps (no force prediction).

## Formulation vs implementation gaps

1. **Missing robot reference priors (j > 0)** — robots at future horizon steps get no pull toward the reference trajectory; only anchored at `j=0`.
2. **Missing centroid velocity regulariser** — `V_j` nodes are unconstrained except through the motion model (regulariser commented out).
3. **Velocity propagation outside FG** — no `V_{j+1} = V_j + F/M * dt` factor between horizon steps; velocity is updated as a scalar post-process after the solve (noted as TODO).
4. **Lambda closure (fragile)** — `robot_nodes_fg` lambdas capture `i` by reference; works coincidentally because all call sites use a `for i` loop with the same variable name.
