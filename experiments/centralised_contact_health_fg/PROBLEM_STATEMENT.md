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

This experiment introduces force sensing at the robot–payload interface as
a *contact-health observation channel* feeding four points in the MR.CAP
control loop: (i) the centroid-pose anchor (via weighted Procrustes), (ii)
the control regulariser (via a $\sigma_u$ modulator, retained as a
contingent safeguard), (iii) a per-robot bidirectional wall-force regulator
applied post-solve, and (iv) a contact-health-gated per-robot position lock
in the estimated centroid frame. The structural question being tested is
**whether per-robot normal-force measurements admit defensible uses that
the pose-only MR.CAP formulation cannot replicate** — without enlarging the
FG variable set beyond MR.CAP's $O(1)$-in-$n$ structure. The motivating
insight is that *only force can evaluate loss of contact*: a pose-only
estimator cannot tell a well-gripping robot from one that has slipped past
the payload, because both look the same in pose.

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
diagnostic of *dragging*. Reducing $\sigma_u^{\text{eff}}$ tightens the
regulariser, pulling commanded velocities back toward zero and relieving the
stress. The factor is one-sided — it only activates for $\bar F_k >
F_{wall}^*$.

**Empirical caveat.** This factor is included for theoretical completeness
and as a safety layer under aggressive operating regimes (heavy payload,
tight reference, external resistive disturbance). In the no-disturbance
translation experiments to date, MR.CAP-style controllers drift to *under*-
squeeze, not over-squeeze: $\bar F_k$ stays well below $F_{wall}^*$ for the
duration of the run, so this factor is dormant. The dominant failure mode
addressed in this work is therefore the *opposite* one — marginal grip —
handled by the per-robot recovery and gated position-lock terms below. The
$\sigma_u$ modulator is retained as a contingent safeguard rather than a
load-bearing component of the contribution.

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
implementation cost), we add a small **post-solve per-robot correction**, a
bidirectional P-controller on each robot's own wall force:
$$
\mathbf{v}_i^{\text{cmd}}
= \mathbf{v}_i^{\text{rigid}}(\mathbf{u}_k^*)
+ \beta \, \bigl(F_{wall}^* - F_{wall,i}\bigr) \, \hat{n}_i
$$
where $\hat{n}_i = [\cos\theta_i, \sin\theta_i]^\top$ is robot $i$'s forward
axis in the world frame (the direction that drives the fork wall *into* the
payload), and $\beta$ is a small gain (default $\beta = 0.005\,\text{m
s}^{-1}\,\text{N}^{-1}$). The sign is automatic:
$F_{wall,i} < F_{wall}^*$ produces $+\hat n_i$ (push into payload, engage);
$F_{wall,i} > F_{wall}^*$ produces $-\hat n_i$ (back away, relieve).
Equilibrium is at $F_{wall,i} \approx F_{wall}^*$.

This term is per-robot and reactive. It violates the rigid-body kinematic
invariant by $O(\beta \Delta t)$ per step, which is intentional: the
formation is allowed to self-correct against the open-loop drift that MR.CAP
ignores. The resulting non-nominal poses are absorbed by the weighted-
Procrustes anchor in the next step (a robot whose $F_{wall,i}$ has fallen
below target is down-weighted in the centroid estimate until it re-engages).

**Sensor caveat (acknowledged).** Because $F_{wall,i} \geq 0$ is a magnitude
of compressive contact, the signal cannot distinguish "robot being pushed
away by payload" (legitimate back-off) from "robot generating necessary
reaction force during a maneuver" (false alarm). The bidirectional response
is therefore only safe in combination with the position-lock layer below,
which re-asserts formation geometry against spurious "back off" commands.
Force-recovery alone, applied bidirectionally, was observed to release the
formation under aggressive maneuvers; the two layers together are stable on
the pure-translation goals targeted in this work.

**Symmetry.** The four contact-health components together form a closed
loop around the rigid-body assumption:

| Regime | Signal | Response |
|--------|--------|----------|
| All healthy | $w_i \approx 1$, $\bar F_k \approx F_{wall}^*$ | Reduces exactly to MR.CAP |
| Per-robot under-squeeze | $F_{wall,i} < F_{wall}^*$ | Force-recovery nudges robot $i$ inward along $\hat n_i$ |
| Per-robot over-squeeze | $F_{wall,i} > F_{wall}^*$ | Force-recovery backs robot $i$ off along $-\hat n_i$ |
| Pose-degraded contact | $w_i \ll 1$ on one or more robots | Weighted anchor de-trusts the affected robot; gated pos-lock pulls it back toward consensus formation |
| Collective over-squeeze | $\bar F_k > F_{wall}^*$ (rare, contingent) | $\sigma_u$ modulator tightens regulariser → collective slowdown |

### Contact-health-gated per-robot position lock

The wall-force recovery above reattaches a slipping robot *physically* but
gives no geometric anchor: the robot may re-engage at a position that is
not its formation slot. We add a small per-robot P loop on position error
in the *estimated* centroid frame, **gated by the same contact-health
weights $w_i$** used by the weighted Procrustes anchor:
$$
\mathbf{p}_i^\text{des} = \hat{\mathbf{p}}_k + R(\hat{\theta}_k)\,\mathbf{r}_i,
\qquad
\mathbf{v}_i^\text{cmd} \mathrel{+}= (1 - w_i)\,K_p\,\bigl(\mathbf{p}_i^\text{des} - \mathbf{p}_i\bigr)
$$
with default $K_p = 1.0$. No new GT dependency — the desired slot is
defined entirely by the weighted-Procrustes output $\hat{\mathbf{c}}_k$
and the (assumed-known) robot poses.

**Why gate by $(1 - w_i)$.** The force-recovery and pos-lock terms encode
two distinct equilibria:
- Force-recovery wants $F_{wall,i} = F_{wall}^*$ — a *physical* equilibrium
  at the radial distance that produces the target compressive force, given
  local geometry and payload deformation.
- Pos-lock wants $\mathbf{p}_i = \mathbf{p}_i^\text{des}$ — a *geometric*
  equilibrium at a predefined slot.

These disagree when the geometric formation slot and the force-equilibrium
slot differ (e.g. the front robot in $n=3$ bears different reaction load
than the side robots). Ungated pos-lock fights force-recovery on the
asymmetric robots and produces visibly uneven grip across the formation.
Gating by $(1 - w_i)$ resolves the conflict: a healthy robot ($w_i \approx
1$) sees gate $\approx 0$ and is left to its force equilibrium; a robot
that has lost contact ($w_i \approx \epsilon$) sees gate $\approx 1$ and is
snapped back toward its slot until it re-engages, at which point its weight
rises and pos-lock fades. Force is the trustworthy signal *when contact
exists*; geometry is the fallback *when force is uninformative*.

**Self-consistency property.** Because the centroid estimate is itself
weighted by contact health, well-gripped robots dominate the consensus
frame; a slipping robot — already down-weighted in $\hat{\mathbf{c}}_k$ —
is commanded back to its slot in the frame the *other* robots collectively
define. The kinematic loop and the estimator close around each other:
robots that maintain grip define "where the formation is"; robots that
lose grip are pulled toward where they should be in that shared frame.
This is the kinematic analogue of $\beta$ — recovery in the position
domain rather than the wall-force domain — and it is something MR.CAP
cannot do, because its open-loop $\mathbf{u}_k \to \mathbf{v}_i^{\text{rigid}}$
map has no per-robot position feedback term.

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

**H1 (grip maintenance, primary).** Over an $(x, y)$ translation goal, the
MR.CAP baseline operates in a *marginal-grip* regime — its rigid-body
kinematic map has no force feedback, so wheel-PD tracking error and payload
reaction accumulate as formation drift, leaving $\bar F_k \ll F_{wall}^*$
for the duration of the run. The contact-health controller maintains
$\bar F_k$ near $F_{wall}^*$ with per-robot distribution within $\sim 25\%$
of the target.

*Metric:* time series of $\bar F_k$ and per-robot $F_{wall,i}$ across the
run; primary statistics are $\text{mean}_k(\bar F_k)$, per-robot mean and
max, and per-robot spread.

**H2 (force-weighted centroid estimation, primary).** From the same robot
poses, the weighted-Procrustes estimator tracks the true payload pose to
within a few centimetres without ground-truth payload feed, where the
unweighted Procrustes estimator inherits any pose-only ambiguity introduced
by formation drift. The argument is structural: **only force can evaluate
loss of contact** — a pose-only estimator cannot distinguish "robot in good
grip at its slot" from "robot slipping past the payload at the same pose",
and so cannot down-weight the latter.

*Metric:* $\|\hat{\mathbf{c}}_k - \mathbf{c}_k^\text{gt}\|$ over the run,
weighted vs unweighted Procrustes from identical robot-pose logs.

**H3 (graceful degradation under disturbance, secondary).** Under a
lateral disturbance applied directly to the payload (`xfrc_applied` pulse,
analogue of an external bump), the contact-health controller maintains
formation grip and recovers trajectory tracking faster than MR.CAP, which
has no observation channel for the disturbance until pose drift becomes
visible in the rigid-body estimate.

*Metric:* recovery time and peak deviation following the disturbance pulse;
per-robot $F_{wall,i}$ traces showing whether grip is maintained throughout
the disturbance.

## Experiment

For $n \in \{3, 4\}$ robots, a face-contact formation (robots engaged on
the cuboid payload faces via forklift contact, see
`generate_mecanum_scene.face_contact_formation`) is used with
`MecanumTransportEnv` (myAGV-class kinematics, MuJoCo physics, force
sensors active). Two translation goals are used: forward $5\,\text{m}$
and diagonal $(3, 2, 0)\,\text{m}$.

Four controller conditions per run, forming an ablation:
1. **MR.CAP baseline** (`mrcap_controller`, unweighted centroid estimator) — control reference.
2. **+ weighted Procrustes only** (estimator swap, otherwise MR.CAP) — isolates H2.
3. **+ weighted Procrustes + σ_u modulator** (FG-level changes only) — isolates the FG-internal contribution.
4. **Full contact-health controller** (weighted Procrustes + σ_u modulator + bidirectional force-recovery + gated pos-lock) — full proposal, tests H1.

Disturbance condition for H3: a lateral $5\,\text{N}$ pulse applied to the
payload for $0.5\,\text{s}$ at $t = 5\,\text{s}$ via `data.xfrc_applied`,
ablated across conditions 1 and 4.

Reported metrics per run:
- **Final position error** $\|\mathbf{c}_\text{final}[:2] - \mathbf{c}_\text{goal}[:2]\|$ (m).
- **Mean trajectory deviation** from the straight-line reference (m).
- **Wall-squeeze**: per-step $\bar F_k$ and per-robot $F_{wall,i}$ time-series;
  per-robot mean, max, and across-robot spread.
- **Centroid estimation error** $\|\hat{\mathbf{c}}_k - \mathbf{c}_k^\text{gt}\|$ (m).
- **FG solve time** per control step: mean, std, max (ms).
- **Wheel torque saturation fraction** (sanity check on tuning).

**Stressors not used.** Wheel-slip injection (per-geom friction scaling on
one robot's wheels) was investigated and found *not* to be a meaningful
stressor in this setup: the forklift–payload contact is the load-bearing
constraint that holds the formation together, so a robot with slipping
wheels simply becomes a passive ride-along dragged forward through the
payload by the other robots. This is recorded explicitly because the
earlier version of this problem statement listed wheel slip as the primary
H2/H3 disturbance — corrected here.

**Logging-only first pass.** Force telemetry suffices to demonstrate H1
without modifying any controller: passing `wall_forces`/`base_forces`
through to the recorder during MR.CAP baseline runs is enough to show the
marginal-grip drift. The full controller is needed only to demonstrate that
grip *can* be maintained.

## Notes on real-lab transfer

All three changes reduce to additional scalar inputs at the controller
boundary ($n$ wall forces, $n$ base forces) — quantities the lab load cells
already produce — plus a per-robot post-solve correction expressed in the
same world-frame velocity command the controller already emits. Nothing
touches the wheel-level control stack or the sim-to-real interface. The
same problem statement is therefore testable identically in MuJoCo and in
the lab, conditional on calibrating $F_{wall}^*$ and $F_{base}^*$ from a
known rest pose.
