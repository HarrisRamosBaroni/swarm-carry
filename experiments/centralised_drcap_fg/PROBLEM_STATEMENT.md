# Problem Statement: Centralised Adaptation of DR.CAP: a Factor-Graph Controller for Multi-Robot Payload Transport

Note: this is mostly the same as the MR.CAP problem statement, but adapted to a centralised verion of DR.CAP.

## Setup

We consider $n$ holonomic (mecanum-wheeled) robots forming a rigid platform that transports
a payload to a goal pose. The robots maintain fixed positions relative to the payload centroid
throughout the motion — the system is treated as a rigid body for the payload, and rigid bodies as well for the robots.

The centroid state is $\mathbf{c}_k = [x_k, y_k, \theta_k]^\top \in \mathbb{R}^3$.
The centroid control is $\mathbf{u}_k = [v_x, v_y, \omega]^\top \in \mathbb{R}^3$ (world-frame
translational velocity and angular rate).
The $i$-th robot state is  $\mathbf{x}_{i_k} = [v_x, v_y, \omega]^\top \in \mathbb{R}^3$(world-frame
translational velocity and angular rate).

Robot $i$'s world-frame velocity follows directly the centroid control:
$$
\mathbf{v}_i = \begin{bmatrix} v_x - \omega \, r_{iy} \\ v_y + \omega \, r_{ix} \end{bmatrix}
$$
so there are no per-robot decision variables — the $2n$ robot velocities are a deterministic
function of the 3-dimensional centroid control $\mathbf{u}_k$. Formation geometry is currently not maintained correctly as not correcly implemented (awaiting Sajad's reponse), so the formation will drift over time, ending in the robots spreading out.
The system will be closed-loop, as the positions of each robot are nodes of the factor graph, being updated at each time step with a direct position measurement simulating a MoCap measurement (note: this 'MoCap measurement is not yet implemented).

## Motion Model

Centroid dynamics are integrated with a world-frame Euler step (exact for holonomic drive):
$$
\mathbf{c}_{k+1} = \mathbf{c}_k + \Delta t \, \mathbf{u}_k $$

## Factor Graph Formulation

At each control step $k$, a receding-horizon factor graph is built over a horizon of $N$ steps.
The decision variables are:
$$
\{ \mathbf{c}_j \}_{j=k}^{k+N}, \quad \{ \mathbf{u}_j \}_{j=k}^{k+N-1} \quad \{ \mathbf{x}_{i_j} \}_{j=k}^{k+N-1}
$$

The graph contains four types of factors:

| Factor | Variables | Role |
|--------|-----------|------|
| Current-state anchor | $\mathbf{c}_k$ | Hard-pins the graph to the measured centroid pose ($\sigma_\text{anchor} = 0.01$) — see note below |
| Reference prior | $\mathbf{c}_j$, $j > k$ | Pulls the trajectory toward a linear-interpolated reference from $\mathbf{c}_k$ to $\mathbf{c}_\text{goal}$ ($\sigma_x = 0.5$) |
| Control regulariser | $\mathbf{u}_j$ | Penalises control effort away from zero ($\sigma_u = 0.3$) |
| Motion model | $(\mathbf{c}_j, \mathbf{u}_j, \mathbf{c}_{j+1})$ | Near-equality enforcing the Euler step ($\sigma_\text{mm} = 10^{-4}$) |
| Terminal anchor | $\mathbf{c}_{k+N}$ | Hard-pins the horizon endpoint to the goal ($\sigma_\text{anchor} = 0.01$) |
| Robot to robot | $(\mathbf{x}_{i_j}, \mathbf{x}_{i+1_j})$ | Penalises variation in distances between 2 robots (maintain initial formation) |
| Pull-in | $(\mathbf{x}_{i_j}, \mathbf{c}_j)$ | Pulls the centroid position towards each robot, to place it in the middle of the formation |

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

(This equation doesn't show up correctly on my .md so I don't really know what to add here, but it's the last 2 factors that are missing)

**Note on $\mathbf{c}_k^\text{meas}$.**
In simulation, the measured centroid pose is obtained directly from the MuJoCo physics state:
position from `data.xpos`, orientation (yaw) from `data.xquat` via quaternion-to-yaw conversion,
and velocities from `data.cvel`. This is noiseless ground-truth. In a real deployment
$\mathbf{c}_k^\text{meas}$ would be the output of a localisation system (e.g. motion capture,
UWB ranging, or an EKF fusing wheel odometry and IMU), and $\sigma_\text{anchor}$ should be
set to reflect that estimator's uncertainty rather than the near-zero value used here.
**Note on that note**
I missed that in the code, I'll have to edit it so that, instead of $\mathbf{c}_k^\text{meas}$ coming from simulation, it should be $\mathbf{x}_{i_k}^\text{meas}$ being measured and added to that graph (same for force factor graph)

This is solved via Levenberg–Marquardt (GTSAM). Because all factors are linear in the
variables and the motion model is linear, the graph is a linear least-squares problem and
LM converges in a single iteration.

Only $\mathbf{u}_0^*$ is extracted from the solution and applied; the remainder of the
optimal trajectory $\{\mathbf{c}_1^*, \ldots, \mathbf{c}_N^*, \mathbf{u}_1^*, \ldots,
\mathbf{u}_{N-1}^*\}$ is discarded. The graph is rebuilt from the new measured state at
the next control step — standard receding-horizon MPC.

## Scalability

The factor graph has $(3+i)(N+1) + 3N$ scalar variables and $(4+i)N + 2$ factors.

## Experiment

For $n \in \{2, 3, 4\}$ robots, a surround formation (robots evenly spaced on a ring around
the payload) is used with `MecanumTransportEnv` (Summit XL Steel, MuJoCo physics).
The payload is transported $5\,\text{m}$ along the $x$-axis from rest.

Reported metrics per run:
- **Final position error** $\|\mathbf{c}_\text{final}[:2] - \mathbf{c}_\text{goal}[:2]\|$ (m)
- **Mean trajectory deviation** from the straight-line reference (m)
- **FG solve time** per control step: mean, std, max (ms)
