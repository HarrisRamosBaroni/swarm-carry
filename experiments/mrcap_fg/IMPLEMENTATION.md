# Implementation Notes: MR.CAP Factor-Graph Controller

This document explains how the factor graph described in `PROBLEM_STATEMENT.md` is
concretely realised in `swarmlib/controllers/mrcap_controller.py` using GTSAM.

## Control actuation chain

The controller outputs an `(n, 2)` array of `[vx, vy]` per robot in the world frame.
`MecanumTransportEnv._apply_controls()` realises these as physical torques via:

1. **World → body frame** rotation using each robot's current yaw
2. **Mecanum IK** mapping body-frame `[vx_b, vy_b]` to target angular velocities for the 4 wheels
3. **PD torque controller** converting wheel velocity error to motor torque:
   `torque = wheel_kv * (target_vel - current_vel)`, `wheel_kv = 200 Nm/(rad/s)` by default
4. Torques written to `data.ctrl` — MuJoCo integrates forward

Commands are therefore **not perfectly tracked**: there is PD lag, and payload reaction
forces acting through the carriage can resist individual robots, causing their actual
velocity to differ from the command. The controller receives no feedback about this
discrepancy.

## GTSAM in one sentence

GTSAM is a C++ library (with Python bindings) that lets you construct a factor graph as
an explicit in-memory data structure and then solve it with a choice of nonlinear optimisers.
You do not manually write the cost function — you describe the graph topology and GTSAM
assembles the sparse Jacobian/Hessian and runs the solver automatically.

## Graph construction

Every control step, `_solve_fg` builds a fresh graph from scratch:

```python
graph = gtsam.NonlinearFactorGraph()   # the graph object
init  = gtsam.Values()                 # initial guess for all variables
```

**Variables** are identified by integer keys created with `gtsam.symbol`:

```python
Ck(j) = gtsam.symbol('C', j)   # centroid pose at horizon step j,  dim=3
Uk(j) = gtsam.symbol('U', j)   # centroid control at step j,        dim=3
```

**Factors** are added as `gtsam.CustomFactor` objects, each specifying:
- a noise model (diagonal Gaussian, sets the weight of that factor in the cost)
- the list of variable keys it connects
- a Python callback that returns the error vector and analytic Jacobians

```python
graph.add(gtsam.CustomFactor(noise_model, [key1, key2, ...], error_fn))
```

The graph therefore exists as a real object in memory — a collection of factor nodes each
holding references to their connected variable keys. It is not just a conceptual aid for
deriving a cost function.

## Initial guess (`init`)

`init` is a `gtsam.Values` dict mapping every variable key to a 3-vector starting point
for the optimiser. It is rebuilt every step from the current reference trajectory `ref`
(linear interpolation from current centroid to goal):

| Variable | Initial guess |
|----------|--------------|
| `C_j` | `ref[j]` — the reference pose at step j |
| `U_j` | `(ref[j+1] - ref[j]) / dt` — finite difference of the reference, i.e. the constant velocity that would traverse it in one step |

`U_j`'s warm-start is therefore derived from the `C_j` warm-start, but both are
**independent decision variables** during the solve — the solver optimises them jointly.

## Joint optimisation and motion model coupling

$\mathbf{C}_j$ and $\mathbf{U}_j$ are not in a leader/follower relationship inside the
solver. The motion model factor

$$
\text{error} = \mathbf{c}_{j+1} - (\mathbf{c}_j + \Delta t\,\mathbf{u}_j)
$$

couples them with a very tight noise model ($\sigma_\text{mm} = 10^{-4}$), effectively
a soft equality constraint. GTSAM enforces this by including it in the global least-squares
cost alongside all other factors — it does not substitute one variable out analytically.

## Solve and receding horizon

```python
result = gtsam.LevenbergMarquardtOptimizer(graph, init, params).optimize()
U_opt  = result.atVector(Uk(0))   # only U_0* is used
```

Only the first control $\mathbf{u}_0^*$ is applied. The rest of the solution
($\mathbf{C}_1^*, \ldots, \mathbf{C}_N^*, \mathbf{u}_1^*, \ldots, \mathbf{u}_{N-1}^*$)
is discarded. There is **no warm-starting from the previous solve** — the graph and `init`
are constructed anew every step at measurement frequency (20 Hz, `dt_control = 0.05 s`).

## Why not warm-start from the previous solution?

The problem is linear (linear motion model, linear factors, Gaussian noise), so LM
converges in a single iteration regardless of the starting point. Warm-starting would save
one matrix assembly but adds code complexity for no practical benefit here.

## Why is the graph object not reused across steps?

The graph topology (which variables each factor connects to) is identical every step, so
in principle the same `NonlinearFactorGraph` could be reused. However, the factor
*measurements* change each step — the anchor at `C_0` updates to the new measured centroid
and `ref` is recomputed. Because `CustomFactor` bakes measurements into the error closure
via `functools.partial`, there is no API to update just the measurement in place; the
factor object itself would need replacing. Since rebuilding the full graph is cheap (the
problem is linear, LM runs one iteration), the simpler approach of constructing a fresh
graph each step was taken. The previous graph object goes out of scope at the end of
`_solve_fg` and is reclaimed by Python's garbage collector.
