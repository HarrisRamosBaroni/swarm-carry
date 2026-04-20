# Problem Statement: Distributed Factor-Graph Controller for Multi-Robot Payload Transport (DR.CAP)

## Setup

We consider $M$ holonomic (mecanum-wheeled) robots forming a rigid platform that transports
a payload to a goal pose. Each robot $R_i$ maintains a **local factor graph** and exchanges
messages only with its neighbours — no single node holds global state.

Robot $i$ has world-frame state $\mathbf{x}_n^{r_i} = [x, y, \theta]^\top \in SE(2)$.
The centroid state is $\mathbf{x}_n^c = [x_c, y_c, \theta_c]^\top$.
The **only decision variable** is the centroid control $\mathbf{u}_n^c = [v_x, v_y, \omega]^\top$;
robot states are a deterministic function of $\mathbf{u}_n^c$ via the rigid-body matrix.

**The payload centroid is unknown.** Each robot maintains a local belief over the shared
centroid trajectory, updated through centroid pull-in and consensus factors.

## Motion Model

Both the centroid and each robot are integrated with a world-frame Euler step (exact for
holonomic drive):

$$
\mathbf{x}_{n+1}^c = \mathbf{x}_n^c + \Delta t \, \mathbf{u}_n^c
$$

$$
\mathbf{x}_{n+1}^{r_i} = \mathbf{x}_n^{r_i} + \Delta t \, M_i \, \mathbf{u}_n^c,
\qquad
M_i = \begin{bmatrix} 1 & 0 & -r_{iy} \\ 0 & 1 & r_{ix} \\ 0 & 0 & 1 \end{bmatrix}
$$

where $\mathbf{r}_i = [r_{ix}, r_{iy}]^\top$ is the fixed offset of robot $i$ from the
centroid (known from formation geometry). Formation shape is maintained by construction.

## Local Factor Graph

Each robot $R_i$ holds a local factor graph over its own trajectory
$\{\mathbf{x}_n^{r_i}\}_{n=0}^{N}$, the shared centroid sequence
$\{\mathbf{x}_n^c\}_{n=0}^{N}$, and the centroid controls
$\{\mathbf{u}_n^c\}_{n=0}^{N-1}$.

The flat variable vector has dimension $9N + 6$ (robot poses, centroid poses, centroid
controls), laid out as $[\mathbf{x}^{r_i}_{0:N},\, \mathbf{x}^c_{0:N},\, \mathbf{u}^c_{0:N-1}]$.

| Factor | Variables | Residual | $\sigma$ |
|--------|-----------|----------|---------|
| Start anchor | $\mathbf{x}_0^{r_i}$, $\mathbf{x}_0^c$ | $\mathbf{x} - \mathbf{x}^\text{meas}$ | $0.01$ |
| Reference prior | $\mathbf{x}_n^c$, $n \in (0, N)$ | $\mathbf{x}_n^c - \mathbf{x}_n^\text{ref}$ | $0.5$ |
| Control regulariser | $\mathbf{u}_n^c$ | $\mathbf{u}_n^c$ | $0.3$ |
| Centroid motion | $(\mathbf{x}_n^c, \mathbf{u}_n^c, \mathbf{x}_{n+1}^c)$ | $\mathbf{x}_{n+1}^c - (\mathbf{x}_n^c + \Delta t\,\mathbf{u}_n^c)$ | $10^{-4}$ |
| Robot motion | $(\mathbf{x}_n^{r_i}, \mathbf{u}_n^c, \mathbf{x}_{n+1}^{r_i})$ | $\mathbf{x}_{n+1}^{r_i} - (\mathbf{x}_n^{r_i} + \Delta t\, M_i\,\mathbf{u}_n^c)$ | $10^{-4}$ |
| Terminal anchor | $\mathbf{x}_N^c$ | $\mathbf{x}_N^c - \mathbf{x}_\text{goal}$ | $0.01$ |
| Centroid pull-in | $(\mathbf{x}_n^{r_i}, \mathbf{x}_n^c)$ | $\mathbf{x}_n^{r_i}[xy] - \mathbf{x}_n^c[xy]$ (xy only; $\theta$ precision = 0) | $0.3$ |
| R2R distance | $(\mathbf{x}_n^{r_i}, \mathbf{x}_n^{r_j})$ | $\|\mathbf{x}_n^{r_i} - \mathbf{x}_n^{r_j}\| - L_{ij}$ (scalar) | $0.05$ |
| Centroid consensus | $\mathbf{x}_n^c$ | $\mathbf{x}_n^c - \hat{\mathbf{x}}_n^{c,\text{nbr}}$ (pulled by neighbour's mean) | $0.1$ |

> **Note:** Obstacle avoidance factors are present in the original DRCAP paper but are not
> implemented here (removed for simplification).

The total cost is:

$$
J = \underbrace{\sum_n e_n^{x,r_i} + e_n^{x,c}}_{\text{anchors}}
  + \underbrace{\sum_{n=1}^{N-1} e_n^\text{ref}}_{\text{reference}}
  + \underbrace{\sum_n e_n^u}_{\text{control}}
  + \underbrace{\sum_n e_n^{m,c} + e_n^{m,r_i}}_{\text{motion}}
  + \underbrace{\sum_n e_n^\text{pull}}_{\text{pull-in}}
  + \underbrace{\sum_{n} \sum_{i \neq j} e_n^{r_ir_j}}_{\text{R2R}}
  + \underbrace{\sum_n e_n^\text{cons}}_{\text{consensus}}
$$

All terms are Mahalanobis-norm squared residuals with the $\sigma$ values listed above.

## Distributed Inference

Rather than a centralised solver, the graph is optimised via **iterative Gaussian Belief
Propagation (GBP)** in canonical (information) form
$\mathbf{x} \sim \mathcal{N}^{-1}(\boldsymbol{\eta}, \boldsymbol{\Lambda})$.

Each GBP iteration:

1. **Re-linearise** the nonlinear R2R distance factor around the current mean.
2. **Apply consensus:** each neighbour's centroid estimate is added as a soft prior
   ($\Lambda \mathbf{x}_n^c += \lambda_\text{cons}\,\hat{\mathbf{x}}_n^{c,\text{nbr}}$).
3. **Linear solve:** $H \boldsymbol{\mu}_\text{new} = \mathbf{b}$ via `np.linalg.solve`.
4. **Broadcast** updated $(\boldsymbol{\eta}_{r_i}, \boldsymbol{\eta}_c)$ to neighbours.
5. **Receive** neighbour messages and unpack into $\hat{\mathbf{x}}^{r_j}$, $\hat{\mathbf{x}}^{c,\text{nbr}}$.

Convergence is declared when $\|\boldsymbol{\mu}_\text{new} - \boldsymbol{\mu}\|_\infty < 10^{-3}$,
with a maximum of 30 iterations per control step.

The linear part of $H$ (all factors except R2R) is precomputed once per control step;
only the R2R Jacobian block is updated each iteration.

Communication is neighbour-only. A configurable backend (simulated, async-with-dropout, or
ZeroMQ) handles message transport. The centroid control $\mathbf{u}_0^{c,*}$ is read from
robot 0's graph and applied to all robots via the rigid-body mapping.

## Experiment

For $n \in \{2, 3, 4\}$ robots, a face-contact formation (robots touching payload faces) is
used with `MecanumTransportEnv` (Summit XL Steel, MuJoCo physics).
The payload is transported $5\,\text{m}$ along the $x$-axis from rest.

Reported metrics per run:
- **Final position error** $\|\mathbf{x}_\text{final}^c[xy] - \mathbf{x}_\text{goal}[xy]\|$ (m)
- **Mean trajectory deviation** from the straight-line reference (m)
- **GBP iterations** per control step: mean, max
- **Solve time** per control step: mean, std, max (ms)
- **Torque saturation fraction** and peak torque (Nm)
