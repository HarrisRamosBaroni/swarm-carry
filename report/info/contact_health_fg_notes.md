# Contact-Health FG Controller — Working Notes for Report

Brief, in-progress notes. Statistics here are illustrative; final figures
and tables to be regenerated systematically from `experiments/centralised_contact_health_fg/`.

## Core finding (the headline)

MR.CAP's open-loop rigid-body kinematic assumption fails *invisibly* in
trajectory metrics but *visibly* in force telemetry. Across $n \in \{3, 4\}$,
pure-translation transport, no disturbance:

- Mean wall-squeeze under MR.CAP: $\bar F \approx 0.7$–$2$ N against a target
  $F^* = 10$ N. At $n=4$, $\bar F = 0.69$ N — the formation is essentially
  *floating around the payload*. This is the IRL "robots slip off payload"
  failure mode quantified.
- Mean wall-squeeze under Contact-Health controller: $\bar F \approx 6$–$8$ N,
  with uniform per-robot distribution. Active grip maintained throughout.
- Trajectory metrics (final position error, mean deviation) are
  **indistinguishable** between baseline and contact-health under no
  disturbance. Force is the differentiating signal, not pose.
- Visually confirmed (user): formation tightness clearly improved under
  contact-health controller; MR.CAP visibly loosens during transport.

## Why force is load-bearing

User observation that crystallises the contribution: **only force info can
evaluate loss of contact.** The pose-only MR.CAP estimator and centroid
fusion cannot distinguish "robot in good contact" from "robot drifted off
the payload" — both manifest as a robot at some pose. Force is the
sufficient observation channel. This argument:

- Motivates the weighted-Procrustes anchor (down-weight robots whose
  force says they're not in contact).
- Motivates the per-robot wall-force regulator (active grip maintenance).
- Argues against orientation-goal evaluation by pose alone: robots can
  successfully *spin and shove* the payload to a target yaw while having
  dropped it; only contact-force telemetry catches this kind of false success.

## Architecture (what landed)

Three contact-health channels layered on top of MR.CAP, **no new FG variables**:

1. **Force-weighted Procrustes anchor.** Centroid pose estimated by
   weighted Procrustes over robot poses; weights
   $w_i = \max(\min(F_{wall,i}/F^*_{wall}, 1) \cdot \min(F_{base,i}/F^*_{base}, 1), \epsilon)$.
   Reduces *exactly* to MR.CAP when all robots are healthy (scale-invariance of
   weighted Procrustes). Centroid estimation accurate to ~1–2 cm
   without ground-truth payload pose.
2. **σ_u modulator.** Tightens control regulariser when mean squeeze
   exceeds target. **Empirically dead under nominal MR.CAP** because
   $\bar F$ never exceeds $F^*$; the failure mode is under-squeeze, not
   over-squeeze. Kept in the formulation in case aggressive maneuvers or
   external disturbances drive collective over-squeeze, but document it
   honestly: it doesn't fire in the no-disturbance experiments to date.
3. **Per-robot wall-force regulator** (post-solve correction).
   $\mathbf{v}_i^{\text{cmd}} = \mathbf{v}_i^{\text{rigid}} + \beta(F^*_{wall} - F_{wall,i}) \hat{n}_i$.
   Bidirectional. Robot pushes into payload when under-pressing, backs off when
   over-pressing. β = 0.005 (current tuning — confirmed visually good).
4. **Contact-health-gated position lock** (post-solve correction).
   $\mathbf{v}_i^{\text{cmd}} \mathrel{+}= (1 - w_i) K_p (\mathbf{p}^{\text{des}}_i - \mathbf{p}_i)$.
   Pulls robot toward its nominal slot in the *estimated* (force-weighted) centroid frame,
   *only when force is uninformative*. Healthy robots ($w_i \approx 1$) are not yanked
   out of their force equilibrium; lost-contact robots ($w_i \approx \epsilon$) snap
   back to formation until they re-engage.

The gating is critical: ungated pos-lock visibly fights force equilibrium
and produces uneven grip (some robots well-engaged, others not). Gated
pos-lock preserves the force-equilibrium for healthy robots, uses geometry
only as a fallback.

## Why bidirectional force-recovery is OK *with* gated pos-lock

A normal-force sensor cannot distinguish "robot being pushed off by payload"
from "necessary reaction force during a maneuver" — both register as
$F_{wall,i} > F^*$. Naive bidirectional force-recovery alone is therefore
unsafe under aggressive maneuvers (observed: catastrophic failure under
yaw-component goals).

Gated pos-lock fixes this by re-asserting formation geometry against the
"back off" command when it disagrees with the consensus pose. Robots can't
drift away from formation even if force-recovery transiently asks them to.
Empirically the combination is stable on pure-translation goals at
β = 0.005, K_p = 1.0.

## What's out of scope

- **Orientation (yaw) goals.** Heading-lock needed so each robot's forklift
  faces the payload while orbiting around a rotating payload — separate
  control loop, hard to tune within the project window. Both MR.CAP and
  contact-health controller drop the payload regularly when given a yaw goal
  in the current sim. Honest framing for the report: "rotation requires a
  heading-lock control layer outside the scope of this formulation; we
  evaluate translation goals only."
- **Yaw lock per robot.** Robots' headings $\theta_i$ drift over time
  because the FG only commands $(v_x, v_y)$. A per-robot yaw P-controller
  driving $\theta_i \to \theta_c + \theta_i^{\text{nominal}}$ would help
  but requires extending the controls signature to $(n, 3)$. Future work.
- **Online mass estimation / 2nd-order dynamics.** Investigated in
  `experiments/centralised_force_fg_cvel/`. Not needed here — sim runs
  with payload mass in [0.2, 50] kg produce visually identical centroid
  trajectories under PD wheel control. Mass-predictive forces don't pay
  for their FG complexity in this regime.
- **Wheel-slip injection as a failure mode.** *Does not work* as a stressor:
  forklift–payload contact (not wheel–floor friction) is the load-bearing
  constraint that holds the formation together. A robot with slipping
  wheels just becomes a passive ride-along dragged forward through the
  payload by the other robots. To stress the controller, use lateral
  `xfrc_applied` on the payload, or reduced forklift–payload friction.

## What to test for the lab data
- Log force time-series under MR.CAP and contact-health controller during
  identical translation tasks. Expect:
  - MR.CAP: $\bar F$ drifts low and uneven; some robots see $\sim 0$ N.
  - Contact-health: $\bar F$ near $F^*$, per-robot distribution within ~25%.
- Capture a clip of MR.CAP dropping the payload (you mentioned this happens
  IRL) and check whether force traces show a single robot's $F_{wall} \to 0$
  before the drop, vs all-at-once collapse. The former is exactly the regime
  the gated-pos-lock catches; the latter is harder.

## Experimental tags for catalogue (current ad-hoc files)

```
experiments/centralised_contact_health_fg/
  gated_n{3,4}_{fwd,diag}_{baseline,full}.json   # β=0.001, F*=10, gated K_p=1.0
  trans_{fwd,diag}_{baseline,full}.json          # β=0.0005, earlier run
  v2_{baseline,weighted,fg,full}.json            # 4-way ablation, β=0.0005, F*=10
  prelim_{baseline,weighted,fg,full}.json        # 4-way ablation, β=0.005, F*=5 — recovery-overshoot regime
  se2_{A,B,C}_{baseline,full}.json               # SE(2) goals — DEPRECATED (orientation out of scope)
  slip_{baseline,full}.json                      # wheel-slip injection — DEPRECATED (wrong stressor)
  hardslip_{baseline,full}.json                  # wheel-slip injection — DEPRECATED
```

When regenerating systematically, suggested naming convention:

```
{controller}_{n}r_{scenario}_{stressor}_{params}.json
e.g. full_4r_fwd5m_nominal_b0.005Fs10Kp1.0.json
     baseline_4r_diag3-2_xfrc_lateral5N_t5s.json
```

with a single `regenerate_all.sh` driver script that emits these names.
Pending lab data we may want different stressor names.

## Open questions / sanity checks to run when time permits

- **β sensitivity sweep**: β ∈ {2e-4, 5e-4, 1e-3, 2e-3, 5e-3}, plot $\bar F$ and
  per-robot std vs β. Tells us how tight the tuning has to be IRL.
- **K_p sensitivity** for gated pos-lock: K_p ∈ {0, 0.5, 1.0, 2.0, 4.0}. K_p=0
  gives no fallback; high K_p risks ringing.
- **xfrc disturbance recovery**: brief lateral kick at t=5s, compare trajectory
  recovery time between MR.CAP and full.
- **σ_u modulator activation regime**: can we contrive a scenario (heavy
  payload + tight reference + aggressive v_max) where $\bar F > F^*$ and the
  modulator actually fires? If not, the writeup should be candid that this
  factor is dormant in our experiments and explain when it would matter.
