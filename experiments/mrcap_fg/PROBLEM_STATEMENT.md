# Problem Statement: Centralised Factor-Graph Controller for Multi-Robot Payload Transport

## Setup

We consider $n$ holonomic (mecanum-wheeled) robots forming a rigid platform that transports
a payload to a goal pose. The robots maintain fixed positions relative to the payload centroid
throughout the motion — the system is treated as a rigid body.

The centroid state is $\mathbf{c}_k = [x_k, y_k, \theta_k]^\top \in \mathbb{R}^3$.
The centroid control is $\mathbf{u}_k = [v_x, v_y, \omega]^\top \in \mathbb{R}^3$ (world-frame
translational velocity and angular rate).

Robot $i$ is located at a fixed offset $\mathbf{r}_i = [r_{ix}, r_{iy}]^\top$ from the centroid
(known from the formation geometry). Its world-frame velocity follows directly from the
rigid-body constraint:
$$
\mathbf{v}_i = \begin{bmatrix} v_x - \omega \, r_{iy} \\ v_y + \omega \, r_{ix} \end{bmatrix}
$$
so there are no per-robot decision variables — the $2n$ robot velocities are a deterministic
function of the 3-dimensional centroid control $\mathbf{u}_k$.

## Motion Model

Centroid dynamics are integrated with a world-frame Euler step (exact for holonomic drive):
$$
\mathbf{c}_{k+1} = \mathbf{c}_k + \Delta t \, \mathbf{u}_k
$$

## Factor Graph Formulation

At each control step $k$, a receding-horizon factor graph is built over a horizon of $N$ steps.
The decision variables are:
$$
\{ \mathbf{c}_j \}_{j=k}^{k+N}, \quad \{ \mathbf{u}_j \}_{j=k}^{k+N-1}
$$

The graph contains four types of factors:

| Factor | Variables | Role |
|--------|-----------|------|
| Current-state anchor | $\mathbf{c}_k$ | Hard-pins the graph to the measured centroid pose ($\sigma_\text{anchor} = 0.01$) |
| Reference prior | $\mathbf{c}_j$, $j > k$ | Pulls the trajectory toward a linear-interpolated reference from $\mathbf{c}_k$ to $\mathbf{c}_\text{goal}$ ($\sigma_x = 0.5$) |
| Control regulariser | $\mathbf{u}_j$ | Penalises control effort away from zero ($\sigma_u = 0.3$) |
| Motion model | $(\mathbf{c}_j, \mathbf{u}_j, \mathbf{c}_{j+1})$ | Near-equality enforcing the Euler step ($\sigma_\text{mm} = 10^{-4}$) |
| Terminal anchor | $\mathbf{c}_{k+N}$ | Hard-pins the horizon endpoint to the goal ($\sigma_\text{anchor} = 0.01$) |

The total cost is:
$$
\min_{\{\mathbf{c}_j\}, \{\mathbf{u}_j\}} \;
\sum_{j=k}^{k+N-1} \left[
  \frac{\|\mathbf{c}_j - \mathbf{c}_j^\text{ref}\|^2}{\sigma_x^2}
  + \frac{\|\mathbf{u}_j\|^2}{\sigma_u^2}
  + \frac{\|\mathbf{c}_{j+1} - \mathbf{c}_j - \Delta t\,\mathbf{u}_j\|^2}{\sigma_\text{mm}^2}
\right]
+ \frac{\|\mathbf{c}_k - \mathbf{c}_k^\text{meas}\|^2}{\sigma_\text{anchor}^2}
+ \frac{\|\mathbf{c}_{k+N} - \mathbf{c}_\text{goal}\|^2}{\sigma_\text{anchor}^2}
$$

This is solved via Levenberg–Marquardt (GTSAM). Because all factors are linear in the
variables and the motion model is linear, the graph is a linear least-squares problem and
LM converges in a single iteration.

## Scalability

The factor graph has $3(N+1) + 3N = 6N + 3$ scalar variables and $4N + 2$ factors,
independent of $n$. Robot count enters only in the analytic post-solve step that maps
$\mathbf{u}_k$ to per-robot velocities — an $O(n)$ operation. Consequently, FG solve time
is $O(1)$ in $n$, in contrast to the MPC formulation (which has $2nT$ decision variables
and $O(n^2)$ Jacobian density, giving solve time scaling $\sim n^\beta$ with $\beta \in [2,3]$).

## Experiment

For $n \in \{2, 3, 4\}$ robots, a surround formation (robots evenly spaced on a ring around
the payload) is used with `MecanumTransportEnv` (Summit XL Steel, MuJoCo physics).
The payload is transported $5\,\text{m}$ along the $x$-axis from rest.

Reported metrics per run:
- **Final position error** $\|\mathbf{c}_\text{final}[:2] - \mathbf{c}_\text{goal}[:2]\|$ (m)
- **Mean trajectory deviation** from the straight-line reference (m)
- **FG solve time** per control step: mean, std, max (ms)
