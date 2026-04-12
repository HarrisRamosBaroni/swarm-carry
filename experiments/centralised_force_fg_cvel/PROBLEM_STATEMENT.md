# Problem Statement: Centralised Force Factor-Graph Controller for Multi-Robot Payload Transport (with un-actuated centroid velocity)

Note: in short, the difference between `\centralised_force_fg` and `\centralised_force_fg_cvel` is that `\centralised_force_fg` conserves the MR.CAP and DR.CAP concept of imagining a control input for the centroid, when `\centralised_force_fg_cvel` instead stores a node for the velocity of the centroid (world frame), that is updated based on the measured forces and the estimtated mass at time step t.

## Setup

We consider $n$ holonomic (mecanum-wheeled) robots forming a rigid platform that transports
a payload to a goal pose. The robots try to maintain fixed positions relative to the payload centroid
throughout the motion — the system is treated as a rigid body for the payload, and rigid bodies as well for the robots.

The centroid state is $\mathbf{c}_k = [x_k, y_k, \theta_k]^\top \in \mathbb{R}^3$.
The centroid velocity is $\mathbf{v}_k = [v_x, v_y, \omega]^\top \in \mathbb{R}^3$ (world-frame
translational velocity and angular rate).
The $i$-th robot state is  $\mathbf{r}_{i_k} = [x_k, y_k, \theta_k]^\top \in \mathbb{R}^3$.
The $i$-th robot control is $\mathbf{u}_{i_k} = [v_x, v_y, \omega]^\top \in \mathbb{R}^3$(world-frame
translational velocity and angular rate).


The system will be closed-loop, as the positions of each robot are nodes of the factor graph, being updated at each time step with a direct position measurement simulating a MoCap measurement.

## Motion Model

Centroid dynamics are integrated with a world-frame 2nd order step:
$$
\mathbf{c}_{k+1} = \mathbf{c}_k + \Delta t \, \mathbf{v}_k + \frac{1}{2} \Delta t ^2 \frac{F}{m} $$

And robot dynamic with a world-frame 1st order (euler) step:
$$
\mathbf{r}_{i_{k+1}} = \mathbf{r}_{i_k} + \Delta t \, \mathbf{u}_{i_k} $$

where $F$ is a $3 \times 1$ vector of the cummulative forces applied to the centroid (currently vertical force is set to 0. We also consider all forces to take effect directly on the center of mass of the centroid, not considering moment arms and torque). (Currently, cummulative force is calculated by summing the world-frame forces of all robots. In the future, forces will be passed by the robots in robot-frame (to match real robots), so the forces will have to be set to world frame first, probably using angle measurement from MoCap directly).
$m$ corresponds to the *mass* node of the factor graph: mass of payload is assumed to be unknown, so we estimate it in the factor graph.

## Factor Graph Formulation

At each control step $k$, a receding-horizon factor graph is built over a horizon of $N$ steps.
The decision variables are:
$$
\{ \mathbf{c}_j \}_{j=k}^{k+N}, \quad \{ \mathbf{v}_j \}_{j=k}^{k+N-1} \quad \{ \mathbf{r}_{i_j} \}_{j=k}^{k+N-1} \quad \{ \mathbf{u}_{i_j} \}_{j=k}^{k+N-1} \quad \{ m \}
$$

The graph contains 11 types of factors (unless I forgot some):

| Factor | Variables | Role |
|--------|-----------|------|
| Current-state anchor | $\mathbf{c}_k$ | Hard-pins the graph to the measured centroid pose ($\sigma_\text{anchor} = 0.01$) — see note below |
| Current-state anchor | $\mathbf{r}_{i_k}$ | Hard-pins the graph to the measured robot poses ($\sigma_\text{anchor} = 0.01$) — see note below |
| Reference prior (centroid) | $\mathbf{c}_j$, $j > k$ | Pulls the trajectory toward a linear-interpolated reference from $\mathbf{c}_k$ to $\mathbf{c}_\text{goal}$ ($\sigma_x = 0.5$) |
| Reference prior (robots)| $\mathbf{r}_{i_j}$, $j > k$ | Pulls the trajectory toward a linear-interpolated reference from $\mathbf{c}_k$ to $\mathbf{c}_\text{goal}$, with a bias to account for the robot's current distance from the centroid ($\sigma_x = 0.5$) |
| Control regulariser (centroid)| $\mathbf{v}_j$ | Penalises control effort of centroid away from zero ($\sigma_u = 0.3$) |
| Control regulariser (robots)| $\mathbf{u}_{i_j}$ | Penalises control effort of robots away from zero ($\sigma_u = 0.3$) |
| Force Motion model | $(\mathbf{c}_j, \mathbf{v}_j, \mathbf{c}_{j+1}, m)$ | Near-equality enforcing the Euler step ($\sigma_\text{mm} = 10^{-4}$) |
| Terminal anchor | $\mathbf{c}_{k+N}$ | Hard-pins the horizon endpoint to the goal ($\sigma_\text{anchor} = 0.01$) |
| Robot to robot | $(\mathbf{r}_{i_j}, \mathbf{r}_{(i+1)_j})$ | Penalises variation in distances between 2 robots (maintain initial formation) |
| Pull-in | $(\mathbf{r}_{i_j}, \mathbf{c}_j)$ | Pulls the centroid position towards each robot, to place it in the middle of the formation |
| Robot Motion model | $(\mathbf{r}_{i_j}, \mathbf{u}_{i_j}, \mathbf{r}_{(i+1)_j} )$ | Near-equality enforcing the Euler step ($\sigma_\text{mm} = 10^{-4}$). Forces are not considered here, only using: $$\mathbf{r}_{i_{k+1}} = \mathbf{r}_{i_k} + \Delta t \, \mathbf{u}_k  $$ |

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

(This equation doesn't show up correctly on my .md so I don't really know what to add here, but it's the new factors that are missing + replacing the old motion model with the new one with forces)

**Note on $\mathbf{c}_k^\text{meas}$ and $\mathbf{r}_{i_k}^\text{meas}$.**
In simulation, the measured centroid pose and robot poses is obtained directly from the MuJoCo physics state:
position from `data.xpos`, orientation (yaw) from `data.xquat` via quaternion-to-yaw conversion,
and velocities from `data.cvel`. This is noiseless ground-truth. In a real deployment
$\mathbf{c}_k^\text{meas}$ and $\mathbf{r}_{i_k}^\text{meas}$ would be the output of a localisation system (e.g. motion capture,
UWB ranging, or an EKF fusing wheel odometry and IMU), and $\sigma_\text{anchor}$ should be
set to reflect that estimator's uncertainty rather than the near-zero value used here.
**Note on that note**
Currently, the code measures $\mathbf{c}_k^\text{meas}$ and $\mathbf{r}_{i_k}^\text{meas}$. In the final code, it may only measure $\mathbf{r}_{i_k}^\text{meas}$, and estimate $\mathbf{c}_k^\text{meas}$ using the factor graph.


This is solved via Levenberg–Marquardt (GTSAM). Because all factors are linear in the
variables and the motion model is linear, the graph is a linear least-squares problem and
LM converges in a single iteration.

Only $\mathbf{u}_{i_0}^*$ are extracted from the solution and applied (however, mass and centroid velocity are also extracted and passed as float values. Eventually, they will be nodes part of a factor graph); the remainder of the
optimal trajectory $\{\mathbf{c}_1^*, \ldots, \mathbf{c}_N^*, \mathbf{v}_1^*, \ldots,
\mathbf{v}_{N-1}^* , \mathbf{r}_{i_1}^*, \ldots, \mathbf{r}_{i_N}^*, \mathbf{u}_{i_1}^*, \ldots,
\mathbf{u}_{i_{N-1}}^*  \}$ is discarded. The graph is rebuilt from the new measured state at
the next control step — standard receding-horizon MPC.
Note that the factor graph is thus completely rebuilt at every time step (this will eventually change).

## Scalability

I did not keep track of the variables/factors, mostly because it's likely to change in the future

## Experiment

For $n \in \{2, 3, 4\}$ robots, a surround formation (robots evenly spaced on a ring around
the payload) is used with `MecanumTransportEnv` (Summit XL Steel, MuJoCo physics).
The payload is transported $5\,\text{m}$ along the $x$-axis from rest.

Reported metrics per run:
- **Final position error** $\|\mathbf{c}_\text{final}[:2] - \mathbf{c}_\text{goal}[:2]\|$ (m)
- **Mean trajectory deviation** from the straight-line reference (m)
- **FG solve time** per control step: mean, std, max (ms)

## TODOs

All TODOs should be written in the docstring at the top of the `force_centralised_controller.py` file (in `swarmlib/controllers/`).

I've however copied them here for accessibility:


- add a way to estimate forces while doing the MPC thing (currently assuming constant): when building the factor graph for the N future states (MPC-like window), forces aren't correctly simulated, instead being modeled as constant (if cumulative forces are F=[1,0,0] at t=0, then F=[1,0,0] for t=1:N)
- remember entire factor graph as opposed to creating a new one every time step: a factor graph is created every time step for generating the control inputs (receding horizon). However, when solved by the optimizer, that factor graph is deleted and a completely new one is created, using measurements from simulation (MoCap equivalents), instead of continuously adding to a large factor graph

- (linked to above TODO) implement mass as a factor graph node initialised a single time at the beginning and that keeps being used and re-estimated as a node (currently mass is only a node that's estimated during the receding horizon, then its final value is passed to the next time step factor graph as an initial value, but that's not the right way of doing it)

- (linked to above TODO) implement centroid velocity as a factor graph node initialised a single time at the beginning and that keeps being used and re-estimated as a node

- Remove real centroid position being fed in from Sim and estimate it instead (use factor graph) (note: get system to work correctly first, then we'll look into that)

- add a high-cost factor in case forces are more than 4-5kg to avoid breaking loadcells ? (so long as we stay in simulation this one isn't really a problem)

- make it decentralised
- ROS implementation
