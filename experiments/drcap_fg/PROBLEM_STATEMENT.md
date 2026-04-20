# Problem Statement: Distributed Factor-Graph Controller for Multi-Robot Payload Transport (DR.CAP)

## Setup

We consider $M$ non-holonomic robots arranged around a rigid payload, tasked with transporting
it from a start position to a goal while avoiding obstacles. Unlike the centralised formulation,
**no single node holds global state**. Each robot $R_i$ maintains a local factor graph and
communicates only with its neighbours.

The state of robot $i$ at time step $n$ is $\mathbf{x}_n^{r_i} = [x, y, \theta]^\top \in SE(2)$.
Control inputs are $\mathbf{u}_n^{r_i} = [v_n^i, \omega_n^i]^\top$ (linear and angular velocity).

**The payload centroid is unknown.** Each robot holds a local belief over a shared centroid
variable $\mathbf{x}_n^c = [x_c, y_c]^\top$, which is estimated collaboratively through
centroid pull-in factors.

## Motion Model

Robot motion is non-holonomic. The nonlinear dynamics are decoupled via a midpoint
approximation. Let $C_\theta = \cos\!\left(\theta_n^{r_i} + \tfrac{\omega_n^{r_i}}{2}\right)$
and $S_\theta = \sin\!\left(\theta_n^{r_i} + \tfrac{\omega_n^{r_i}}{2}\right)$:

$$
\mathbf{x}_{n+1}^{r_i} = \mathbf{x}_n^{r_i} +
\begin{bmatrix} C_\theta & 0 \\ S_\theta & 0 \\ 0 & 1 \end{bmatrix}
T_s \, \mathbf{u}_n^{r_i}
$$

## Factor Graph Formulation

Each robot $R_i$ holds a **local factor graph** over its own trajectory
$\{x_n^{r_i}\}_{n=0}^{N}$, its controls $\{u_n^{r_i}\}_{n=0}^{N-1}$,
and a shared centroid sequence $\{x_n^c\}_{n=0}^{N}$.
Inter-robot factors couple adjacent local graphs.

| Factor | Variables | Role |
|--------|-----------|------|
| Motion ($f_n^m$) | $(\mathbf{x}_n^{r_i}, \mathbf{u}_n^{r_i}, \mathbf{x}_{n+1}^{r_i})$ | Enforces the nonlinear motion model |
| Anchor ($f_n^x$) | $\mathbf{x}_n^{r_i}$ | Pins start and goal positions |
| Obstacle avoidance ($f_n^\text{obs}$) | $\mathbf{x}_n^{r_i}$ | Penalises proximity to obstacles within radius $R$ |
| Robot-to-robot ($f_n^{r_ir_j}$) | $(\mathbf{x}_n^{r_i}, \mathbf{x}_n^{r_j})$ | Maintains inter-robot separation $L_{ij}$ |
| Centroid pull-in ($f_n^{cr_i}$) | $(\mathbf{x}_n^{r_i}, \mathbf{x}_n^c)$ | Each robot contributes equally to estimating the geometric centroid |

The total cost is:

$$
J(\mathbf{x}, \mathbf{u}) =
\sum_{n=k}^{N} e_n^x
+ \sum_{n=k}^{N-1} e_n^m
+ \sum_{j=1}^{J} \sum_{n=k+1}^{N} e_n^\text{obs}
+ \sum_{n=k}^{N-1} \sum_{i \neq j} e_n^{r_ir_j}
+ \sum_{n=k}^{N} \sum_{i} e_n^{cr_i}
$$

where all terms are Mahalanobis-norm squared residuals weighted by per-factor information
matrices $\Omega$ (see Table I of the paper for tuned covariance values).

## Inference: Gaussian Belief Propagation

Rather than solving the factor graph with a centralised solver (e.g. Levenberg–Marquardt),
**Gaussian Belief Propagation (GBP)** is used. All beliefs are maintained in canonical
(information) form $\mathbf{x} \sim \mathcal{N}^{-1}(\boldsymbol{\eta}, \boldsymbol{\Lambda})$.

GBP iterates three message-passing steps until convergence:

1. **Variable → factor:** $m_{x_i \to f_j} = \prod_{s \in \mathcal{N}(i) \setminus j} m_{f_s \to x_i}$
2. **Factor → variable:** $m_{f_j \to x_i} = \sum_{X_j \setminus x_i} f_j(X_j) \prod_{k \in \mathcal{N}(j) \setminus i} m_{x_k \to f_j}$
3. **Belief update:** $b_i(x_i) = \prod_{s \in \mathcal{N}(i)} m_{f_s \to x_i}$

Convergence is declared when the estimated trajectory does not change for 10 consecutive
iterations. Only adjacent robots exchange messages; there is no central coordinator.

The centroid node is held by one robot, with backups distributed across the team.
If the holder drops out, a new robot is elected automatically.

## Distributed Execution

At each control step:

1. Each robot computes its local graph and iterates GBP until local convergence.
2. Beliefs are propagated to neighbours, updating inter-robot and centroid factors.
3. Once all local graphs converge, the planned trajectory is executed for one step.
4. Beliefs are updated from observed robot positions, and the loop repeats.

## Scalability

The local graph for robot $i$ has $O(N)$ variables and factors, independent of $M$.
Communication is neighbour-only ($O(1)$ messages per robot per iteration).
As team size increases from 2 to 64, convergence iterations grow by only $1.1\times$,
demonstrating near-constant scaling in $M$.

## Experiment

For $M \in \{4, 8, 16\}$ robots, trajectories are planned from $(0, 0)$ m to $(10, 0)$ m
in the presence of point obstacles. Results are compared against a centralised
GTSAM/Levenberg–Marquardt baseline.

Reported metrics per run:
- **Inter-robot error** — maximum pairwise deviation from the desired formation (m)
- **Distance to goal** — final centroid error (m)
- **Deviation** — mean deviation of centroid trajectory from A\* optimal path (m)
- **Smoothness** — RMS jerk magnitude over the trajectory ($\text{m}\,\text{s}^{-3}$)
- **Iterations to convergence** — averaged across all robots
