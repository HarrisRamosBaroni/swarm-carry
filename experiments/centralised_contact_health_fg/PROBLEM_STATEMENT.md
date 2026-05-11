# Problem Statement: Centralised Contact-Health Factor-Graph Controller for Multi-Robot Payload Transport

## Motivation

MR.CAP (Jaafar 2024) and its derivatives (`experiments/mrcap_fg/`) treat the
robot–payload interface as a *rigid kinematic constraint*: per-robot velocities
are derived deterministically from a centroid control via the formation
geometry. This is an **open-loop assumption** — the controller does not
observe whether the contact is actually maintained, whether the formation is
being internally stressed (robots dragging each other through the payload),
or whether one robot has lost contact entirely. Per-robot reaction forces at
the contact interface are physically present at all times but never enter
the factor graph.

This experiment introduces force sensing at the robot–payload interface as a
*contact-health observation channel* feeding two points in the MR.CAP factor
graph: the centroid-pose anchor and the control regulariser. The structural
question being tested is **whether per-robot normal-force measurements admit
defensible, FG-native uses that the pose-only MR.CAP formulation cannot
replicate** — without enlarging the FG variable set beyond MR.CAP's
$O(1)$-in-$n$ structure.

## Setup

We consider $n$ holonomic (mecanum-wheeled) robots forming a rigid platform
that transports a payload to a goal pose. The robots maintain *nominal*
fixed positions relative to the payload centroid; deviations from the nominal
formation are no longer ignored.

The centroid state is $\mathbf{c}_k = [x_k, y_k, \theta_k]^\top \in \mathbb{R}^3$.
The centroid control is $\mathbf{u}_k = [v_x, v_y, \omega]^\top \in \mathbb{R}^3$
(world-frame translational velocity and angular rate).

Robot $i$ has *nominal* body-frame offset $\mathbf{r}_i = [r_{ix}, r_{iy}]^\top$
from the centroid. Per-robot world-frame velocities are derived from the
rigid-body kinematic map exactly as in MR.CAP:
$$
\mathbf{v}_i = \begin{bmatrix} v_x - \omega \, r_{iy} \\ v_y + \omega \, r_{ix} \end{bmatrix}
$$
so the FG has no per-robot decision variables. The novelty is not in the
variable structure — it is in how *measurements* enter the graph.

### Contact sensing

Each robot carries two load cells at the forklift-style payload-engagement
interface (see `docs/scenario.md`):

- $F_{base,i} \geq 0$: normal force on the **fork base** (vertical, weight-bearing).
- $F_{wall,i} \geq 0$: normal force on the **fork wall** (horizontal, along
  robot $i$'s forward axis $\hat{n}_i = [\cos\theta_i, \sin\theta_i]^\top$ in
  the world frame).

Both are non-negative — they only register compressive contact. Sensor
geometry is fully described by `swarmlib/simulation/mecanum_env.py` (lines
192–318); in the real lab the analogue is a pair of load cells per robot,
read independently of which controller is running.

Two nominal contact targets, calibrated from a known equilibrium pose with
the payload at rest in the formation:

- $F_{base}^* = m_{nom} \, g / n$ (per-robot share of payload weight)
- $F_{wall}^*$: a chosen squeeze setpoint (e.g. $5$–$10\,\text{N}$) that
  guarantees frictional engagement without saturating motors.

## Motion Model

Unchanged from MR.CAP — world-frame Euler step:
$$
\mathbf{c}_{k+1} = \mathbf{c}_k + \Delta t \, \mathbf{u}_k
$$
The 2nd-order, force-driven dynamics explored in
`experiments/centralised_force_fg_cvel/` are deliberately *not* adopted here.
That formulation requires online mass estimation and turns force into a
predictive quantity — sim experiments with the kinematic MR.CAP baseline
already show that payload mass in $[0.2, 50]\,\text{kg}$ produces visually
identical centroid trajectories under PD wheel control, so a mass-adaptive
predictive force model does not pay for its complexity in this regime. Here
forces enter as **measurements that update belief**, not as predictive
dynamics.

## Factor Graph Formulation

At each control step $k$, a receding-horizon factor graph is built over a
horizon of $N$ steps. The decision variables are MR.CAP-identical:
$$
\{ \mathbf{c}_j \}_{j=k}^{k+N}, \quad \{ \mathbf{u}_j \}_{j=k}^{k+N-1}
$$

The graph contains five factor types; only two of MR.CAP's anchor/regulariser
factors are modified, and no factors are added or removed. The structural
$O(1)$-in-$n$ property is preserved.

| Factor | Variables | Role |
|--------|-----------|------|
| **Contact-weighted current-state anchor** | $\mathbf{c}_k$ | Pins the graph to a centroid pose $\hat{\mathbf{c}}_k$ estimated by **force-weighted Procrustes** over robot poses (see below) ($\sigma_\text{anc} = 0.01$) |
| Reference prior | $\mathbf{c}_j$, $j > k$ | Pulls trajectory toward linear-interpolated reference from $\hat{\mathbf{c}}_k$ to $\mathbf{c}_\text{goal}$ ($\sigma_x = 0.5$) |
| **Contact-health-modulated control regulariser** | $\mathbf{u}_j$ | Penalises control effort with a contact-stress-aware standard deviation $\sigma_u^{\text{eff}}(\bar F_k)$ (see below) |
| Motion model | $(\mathbf{c}_j, \mathbf{u}_j, \mathbf{c}_{j+1})$ | Euler step ($\sigma_\text{mm} = 10^{-4}$) |
| Terminal anchor | $\mathbf{c}_{k+N}$ | Hard-pins horizon endpoint to goal ($\sigma_\text{anc} = 0.01$) |

### Force-weighted Procrustes (replaces equal-weight estimator)

The current `swarmlib/controllers/centroid_estimator.py` computes
$\hat{\mathbf{c}}_k$ from robot poses via unweighted orthogonal Procrustes —
every robot pose contributes equally regardless of whether that robot is
actually in good contact with the payload. This is the assumption MR.CAP
inherits implicitly.

We replace it with a **weighted Procrustes** estimator. Given current robot
positions $\mathbf{p}_i$, nominal body-frame offsets $\mathbf{r}_i$, and
per-robot contact-health weights $w_i \in [0, 1]$:

$$
\min_{R \in SO(2),\, \mathbf{t}} \quad \sum_{i=1}^{n} w_i \,
\| \mathbf{p}_i - (R \mathbf{r}_i + \mathbf{t}) \|^2
$$

with closed-form solution via weighted-mean centring and weighted SVD:
$\bar{\mathbf{p}} = \frac{\sum_i w_i \mathbf{p}_i}{\sum_i w_i}$,
$\bar{\mathbf{r}} = \frac{\sum_i w_i \mathbf{r}_i}{\sum_i w_i}$,
$M = \sum_i w_i (\mathbf{p}_i - \bar{\mathbf{p}})(\mathbf{r}_i - \bar{\mathbf{r}})^\top$,
$M = U \Sigma V^\top$, $R = U \, \text{diag}(1, \det(UV^\top)) \, V^\top$,
$\mathbf{t} = \bar{\mathbf{p}} - R \bar{\mathbf{r}}$. The yaw is recovered as
$\hat{\theta}_k = \text{atan2}(R_{10}, R_{00})$.

Weights are derived from both load cells and clamped to $[\epsilon, 1]$:
$$
w_i = \max\!\left(\, \min\!\left(\frac{F_{wall,i}}{F_{wall}^*},\, 1\right) \cdot
      \min\!\left(\frac{F_{base,i}}{F_{base}^*},\, 1\right),\; \epsilon \,\right)
$$
with $\epsilon = 10^{-3}$. Saturating the ratios at $1$ from above prevents
an over-pressing robot from dominating the estimate; the floor at $\epsilon$
prevents the system from becoming rank-deficient when all robots
simultaneously lose contact. Note that weighted Procrustes is invariant
under a uniform scaling of the weights — only their *relative* magnitudes
matter — so the floor is operationally a regulariser, not a bias.

**Interpretation.** A robot with degraded contact (one or both forces near
zero) contributes near-$\epsilon$ weight; its pose may be far from the
nominal offset due to slip but should not bias the centroid estimate. A
robot at or above nominal contact contributes weight $1$. In the all-healthy
case $w_i = 1\ \forall i$, and the weighted estimator reduces *exactly* to
the unweighted MR.CAP estimator (by scale invariance). This change is
therefore **strictly an extension**, not a substitution.

### Contact-health-modulated regulariser

The MR.CAP control regulariser uses a fixed $\sigma_u$. We modulate it by the
mean wall-squeeze residual:
$$
\bar F_k = \frac{1}{n} \sum_{i=1}^{n} F_{wall,i}, \qquad
h_k^+ = \max(\bar F_k - F_{wall}^*, \, 0)
$$
$$
\sigma_u^{\text{eff}}(\bar F_k) = \frac{\sigma_u^0}{1 + \alpha \, h_k^+}
$$
with $\sigma_u^0 = 0.3$ (MR.CAP default) and $\alpha$ a tuning constant
(default $\alpha = 0.1\,\text{N}^{-1}$).

**Interpretation.** When $\bar F_k$ exceeds the nominal squeeze setpoint, the
robots are collectively pressing harder into the payload than necessary —
diagnostic of *dragging*, i.e. the formation is internally stressed because
robots are commanding forward velocities the payload can't sustain (likely
due to wheel-PD undershoot or transient external resistance). Reducing
$\sigma_u^{\text{eff}}$ tightens the regulariser, pulling commanded velocities
back toward zero and relieving the stress.

The factor is **asymmetric** — it only activates for $\bar F_k > F_{wall}^*$.
Below-target squeeze cannot be fixed by slowing the formation down; it
requires *per-robot inward motion*, which is structurally outside MR.CAP's
variable set (robot velocities are a deterministic function of the centroid
control $\mathbf{u}_k$). The complementary action is handled outside the
factor graph by the per-robot contact-recovery term defined below.

The factor cost is:
$$
\sum_{j=k}^{k+N-1} \frac{\|\mathbf{u}_j\|^2}{(\sigma_u^{\text{eff}}(\bar F_k))^2}
$$
Note that $\sigma_u^{\text{eff}}$ depends only on the *current* measurement
$\bar F_k$ and is held constant across the horizon — there is no force
prediction. This preserves linearity of the factor in $\mathbf{u}_j$.

### Per-robot contact-recovery (post-solve correction)

The two FG-level changes above address *excess* squeeze (collective
slowdown via the regulariser) and *degraded-but-unknown* contact (down-
weighted poses in the anchor). Neither restores marginal contact —
that requires individual robots to move inward toward the payload, which
the rigid-body kinematic map $\mathbf{u}_k \to \mathbf{v}_i^{\text{rigid}}$
cannot express because all per-robot velocities are tied to a single
centroid control.

Rather than expanding the FG variable set to per-robot decisions (which is
the direction `experiments/centralised_force_fg_cvel/` takes, with substantial
implementation cost), we add a small **post-solve per-robot correction**:
$$
\mathbf{v}_i^{\text{cmd}}
= \mathbf{v}_i^{\text{rigid}}(\mathbf{u}_k^*)
+ \beta \, \bigl(F_{wall}^* - F_{wall,i}\bigr)^+ \, \hat{n}_i
$$
where $\hat{n}_i = [\cos\theta_i, \sin\theta_i]^\top$ is robot $i$'s forward
axis in the world frame (the direction that drives the fork wall *into* the
payload), $(\cdot)^+ = \max(\cdot, 0)$, and $\beta$ is a small gain (default
$\beta = 0.005\,\text{m s}^{-1} \text{N}^{-1}$, i.e. a $20\,\text{N}$
under-squeeze produces a $0.1\,\text{m/s}$ inward correction).

This term is per-robot and reactive — it acts only on robots whose own
$F_{wall,i}$ has fallen below target. It violates the rigid-body kinematic
invariant by $O(\beta \Delta t)$ per step, which is intentional: the
formation is allowed to self-correct against the open-loop drift that MR.CAP
ignores. The resulting non-nominal poses are absorbed by the weighted-
Procrustes anchor in the next step (a robot mid-recovery has $F_{wall,i} <
F_{wall}^*$ and so its pose is down-weighted in the centroid estimate
until it re-engages).

**Symmetry.** The three changes together form a symmetric closed loop
around the rigid-body assumption:

| Regime | Signal | Response |
|--------|--------|----------|
| All healthy | $w_i \approx 1$, $\bar F_k \approx F_{wall}^*$ | Reduces exactly to MR.CAP |
| Excess squeeze | $\bar F_k > F_{wall}^*$ | Regulariser shrinks $\sigma_u^{\text{eff}}$ → collective slowdown |
| Pose-degraded contact | $w_i \ll 1$ on one or more robots | Weighted anchor de-trusts the affected robot's pose |
| Marginal squeeze | $F_{wall,i} < F_{wall}^*$ on one or more robots | Per-robot $\hat{n}_i$ correction nudges the robot inward |

### Per-robot position lock (kinematic counterpart to recovery)

The wall-force recovery above reattaches a slipping robot *physically* but
gives no geometric anchor: the robot may re-engage at a position that is
not its formation slot. We add a small per-robot P loop on position error
in the *estimated* centroid frame:
$$
\mathbf{p}_i^\text{des} = \hat{\mathbf{p}}_k + R(\hat{\theta}_k)\,\mathbf{r}_i,
\qquad
\mathbf{v}_i^\text{cmd} \mathrel{+}= K_p\,\bigl(\mathbf{p}_i^\text{des} - \mathbf{p}_i\bigr)
$$
with default $K_p = 2.0$. No new GT dependency — the desired slot is
defined entirely by the weighted-Procrustes output $\hat{\mathbf{c}}_k$
and the (assumed-known) robot poses.

**Self-consistency property.** Because the centroid estimate is itself
weighted by contact health, well-gripped robots dominate the consensus
frame; a slipping robot — already down-weighted in $\hat{\mathbf{c}}_k$ —
is then commanded back to its slot in the frame the *other* robots
collectively define. The kinematic loop and the estimator close around
each other: bots that maintain grip define "where the formation is";
bots that lose grip are pulled toward where they should be in that
shared frame. This is the kinematic analogue of $\beta$ — recovery in
the position domain rather than the wall-force domain — and it is
something MR.CAP cannot do, because its open-loop $\mathbf{u}_k \to
\mathbf{v}_i^{\text{rigid}}$ map has no per-robot position feedback term.

### Orientation goals — out of scope

Both this controller and MR.CAP currently target $(x, y)$ goals only. The
factor graph admits a $\theta$ goal trivially, but the *physical* execution
runs into a load-asymmetry failure mode: orbital translation
($\mathbf{v} = \boldsymbol\omega \times \mathbf{r}_i$) is loaded by the
payload reaction, while own-axis spin is unloaded, so per-robot wheel-PD
tracking of the two diverges and the fork loses tidal lock with the
payload face. Rigid attachment (as in the original MR.CAP paper) hides
this by mechanical constraint; the forklift contact does not. Closing it
requires a heading-lock loop analogous to $K_p$ above and careful
wheel-allocator coupling — left as future work. Until then, all
$\omega$-goal terms are dropped from the reference and per-robot commands
contain no yaw rate.

### Total cost

$$
\min_{\{\mathbf{c}_j\}, \{\mathbf{u}_j\}} \;
\sum_{j=k}^{k+N-1} \left[
  \frac{\|\mathbf{c}_j - \mathbf{c}_j^\text{ref}\|^2}{\sigma_x^2}
  + \frac{\|\mathbf{u}_j\|^2}{(\sigma_u^{\text{eff}}(\bar F_k))^2}
  + \frac{\|\mathbf{c}_{j+1} - \mathbf{c}_j - \Delta t\,\mathbf{u}_j\|^2}{\sigma_\text{mm}^2}
\right]
+ \frac{\|\mathbf{c}_k - \hat{\mathbf{c}}_k\|^2}{\sigma_\text{anc}^2}
+ \frac{\|\mathbf{c}_{k+N} - \mathbf{c}_\text{goal}\|^2}{\sigma_\text{anc}^2}
$$

where $\hat{\mathbf{c}}_k$ and $\sigma_u^{\text{eff}}(\bar F_k)$ are functions
of the current sensor reading $(\{F_{wall,i}\}, \{F_{base,i}\})$ but constants
within a single solve. The graph is therefore still a linear least-squares
problem and LM converges in one iteration.

Only $\mathbf{u}_0^*$ is extracted from the solve, mapped through the
rigid-body kinematic to per-robot $\mathbf{v}_i^{\text{rigid}}$, augmented
with the per-robot contact-recovery correction above, and applied; the rest
of the horizon is discarded; the graph is rebuilt next step from a fresh
$(\hat{\mathbf{c}}_k, \bar F_k)$ — standard receding-horizon MPC,
identical in structure to MR.CAP.

## Scalability

Variables and factors per solve: $3(N+1) + 3N = 6N + 3$ scalars and $4N + 2$
factors — identical to MR.CAP, independent of $n$. The weighted-Procrustes
pre-step adds $O(n)$ work outside the solve (one weighted SVD on a $2{\times}2$
matrix). The squeeze-residual computation is $O(n)$. Total per-step overhead
beyond MR.CAP is $O(n)$, dominated by the same post-solve $\mathbf{u}_k \to
\{\mathbf{v}_i\}$ mapping that MR.CAP already pays. **Asymptotic complexity
is unchanged.**

## Hypotheses

**H1 (formation-stress regulation, primary).** Over a $5\,\text{m}$ straight
transport, the unweighted MR.CAP baseline produces unbounded drift in mean
wall-squeeze $\bar F_k$ over time (consequence of accumulated wheel-PD
tracking error against the open-loop kinematic constraint). The contact-health
controller maintains $\bar F_k$ in a bounded band around $F_{wall}^*$.

*Metric:* time series of $\bar F_k$ across the run; primary statistic is
$\max_k \bar F_k - F_{wall}^*$ and $\text{std}_k(\bar F_k)$.

**H2 (force-weighted centroid estimation, primary).** Under induced contact
degradation on one robot (low wheel friction patch from $t = t_\text{slip}$
onward, simulated via per-geom friction override), the weighted-Procrustes
estimator yields lower centroid pose error than the unweighted estimator,
because the slipping robot's pose ceases to be representative.

*Metric:* $\|\hat{\mathbf{c}}_k - \mathbf{c}_k^\text{gt}\|$ averaged over
$t \geq t_\text{slip}$; comparison between weighted and unweighted estimators
*on the same trajectory* (no controller change), then end-to-end with the
contact-health controller closing the loop.

**H3 (active contact recovery, secondary).** Under induced contact
degradation, the post-solve per-robot recovery term restores $F_{wall,i}$
toward $F_{wall}^*$ on the affected robot (re-engagement) without requiring
formation reconfiguration through the FG. With all three changes active,
the contact-health controller completes the $5\,\text{m}$ transport with
lower final position error than the MR.CAP baseline under the induced-slip
disturbance, despite using a degraded centroid estimate during the recovery
window.

*Metric:* $\|\mathbf{c}_\text{final}[:2] - \mathbf{c}_\text{goal}[:2]\|$.

## Experiment

For $n \in \{3, 4\}$ robots, a surround formation (robots evenly spaced on a
ring around the payload, forklifts engaged) is used with `MecanumTransportEnv`
(myAGV-class kinematics, MuJoCo physics, force sensors active). The payload
is transported $5\,\text{m}$ along the $x$-axis from rest.

Four controller conditions per run, forming an ablation:
1. **MR.CAP baseline** (`mrcap_controller`, unweighted centroid estimator) — control reference.
2. **+ weighted Procrustes only** (estimator swap, otherwise MR.CAP) — isolates H2.
3. **+ weighted Procrustes + contact-health regulariser** (FG-level changes only, no per-robot recovery) — isolates H1.
4. **Full contact-health controller** (all three changes active) — full proposal, tests H3.

The ablation lets each change be credited or discredited independently.

Two scenarios per condition:
- **A. Nominal:** all robots at full friction; tests H1.
- **B. Induced slip:** at $t = 5\,\text{s}$, friction on one robot's wheels
  is reduced by an order of magnitude for $2\,\text{s}$; tests H2 and H3.

Reported metrics per run:
- **Final position error** $\|\mathbf{c}_\text{final}[:2] - \mathbf{c}_\text{goal}[:2]\|$ (m).
- **Mean trajectory deviation** from the straight-line reference (m).
- **Mean / max wall-squeeze residual** $\bar F_k - F_{wall}^*$ over time (N).
- **Centroid estimation RMSE** $\|\hat{\mathbf{c}}_k - \mathbf{c}_k^\text{gt}\|$ (m, rad).
- **FG solve time** per control step: mean, std, max (ms).

The H1 plot (squeeze drift under MR.CAP) and the H2 plot (estimator-error
divergence under slip) are both **logging-only**: they can be produced from
runs of the existing MR.CAP baseline by passing `wall_forces`/`base_forces`
through to the recorder without modifying the controller. This gives the
empirical anchor for the proposed contribution before the new controller is
implemented.

## Notes on real-lab transfer

All three changes reduce to additional scalar inputs at the controller
boundary ($n$ wall forces, $n$ base forces) — quantities the lab load cells
already produce — plus a per-robot post-solve correction expressed in the
same world-frame velocity command the controller already emits. Nothing
touches the wheel-level control stack or the sim-to-real interface. The
same problem statement is therefore testable identically in MuJoCo and in
the lab, conditional on calibrating $F_{wall}^*$ and $F_{base}^*$ from a
known rest pose.
