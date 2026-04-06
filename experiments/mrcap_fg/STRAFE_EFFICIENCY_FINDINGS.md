# Mecanum Strafe Efficiency: Investigation & Fix

## Problem

Robots in a surround formation (mixed orientations) drift apart during transport.
Robots that must strafe (body-frame lateral motion) achieve only ~55% of the
displacement of robots that drive forward, regardless of commanded speed.

This breaks the rigid-body formation assumption used by the FG controller.

## Root Cause

**The ~55% strafe efficiency is a fundamental property of the mecanum roller
contact model in MuJoCo, not actuator torque saturation.**

### Evidence: ctrlrange sweep (`diagnose_strafe_efficiency.py`)

Swept actuator torque limit (10--200 Nm) across 6 command speeds (0.01--1.0 m/s).
The strafe/forward displacement ratio is flat across all torque limits:

```
Strafe/Forward ratio matrix  (rows=cmd_speed, cols=ctrlrange)
  cmd(m/s)      10Nm      25Nm      50Nm     100Nm     200Nm
     0.050   0.5513   0.5456   0.5940   0.6911   0.5454
     0.100   0.5525   0.5423   0.5662   0.5451   0.5480
     0.250   0.5562   0.5516   0.5534   0.5498   0.5504
     0.500   0.5525   0.5534   0.5472   0.5391   0.5058
     1.000   0.5236   0.5417   0.5455   0.5361   0.5158
```

(The 0.01 m/s row is noisy due to sub-millimetre displacements — excluded.)

If torque saturation were the cause, increasing ctrlrange would improve the ratio.
It does not. The roller contact mechanics (passive roller friction, slip, compliance)
are the limiting factor.

### Why sat% was misleading

The PD gain `_wheel_kv = 200 Nm/(rad/s)` is very aggressive. Saturation threshold
is only `10 / 200 = 0.05 rad/s` velocity error. Any transient disturbance
(roller contact bumps) briefly triggers saturation, producing high sat% readings
even at very low commanded speeds. But steady-state speed scales linearly with
command — confirming no sustained saturation.

## Fix: Per-Robot Velocity Feedback Loop

Added an optional PI controller in `MecanumTransportEnv.step()` that compares
each robot's commanded world-frame velocity to its actual velocity (from
`data.cvel`) and adjusts the command before the wheel IK chain.

```
vel_error = commanded_vel - actual_vel       # world frame, per robot
integral += vel_error * dt
corrected = commanded + Kp * error + Ki * integral
```

Parameters (in `MecanumTransportEnv.__init__`):
- `vel_feedback=True` to enable
- `vel_fb_kp=2.0` (proportional gain)
- `vel_fb_ki=5.0` (integral gain)
- `vel_fb_integral_max=2.0` (anti-windup clamp, m/s)

### Verification (`diagnose_vel_feedback.py`)

Surround formation, 200 steps, 4 command speeds:

```
cmd(m/s)  |  NO FB ratio  |  WITH FB ratio
  0.050   |     0.5518     |     0.9876
  0.100   |     0.5520     |     0.9760
  0.250   |     0.5517     |     0.9824
  0.500   |     0.5456     |     0.9786
```

The feedback loop raises the strafe/forward ratio from ~0.55 to ~0.98 across
all tested speeds. The remaining ~2% gap is residual integral lag over the
finite run length.

### Why feedback over feedforward compensation

A feedforward approach (dividing body-frame vy by eta=0.55) was considered but
rejected:

1. eta is specific to this URDF + MuJoCo contact model — brittle to changes
2. Feedback corrects any tracking error (payload drag, floor friction, wear),
   not just strafe inefficiency
3. Matches what a real system would do with odometry/tracking
4. Does not require a magic constant

## Files

| File | Purpose |
|------|---------|
| `diagnose_strafe_efficiency.py` | ctrlrange × speed sweep — proves roller contact is root cause |
| `diagnose_vel_feedback.py` | Verifies PI feedback fixes the ratio |
| `diagnose_formation.py` | Original formation drift diagnostic (no payload) |
| `figures/diagnose_strafe_efficiency.png` | Ratio vs ctrlrange plot |
| `figures/diagnose_vel_feedback.png` | Feedback on/off comparison |

## Usage

To enable velocity feedback in experiments, pass `vel_feedback=True` when
constructing `MecanumTransportEnv`:

```python
env = MecanumTransportEnv(
    n_robots=n_robots,
    formation=formation,
    vel_feedback=True,      # enables PI velocity correction
    # vel_fb_kp=2.0,        # defaults are tuned for Summit XL Steel
    # vel_fb_ki=5.0,
    ...
)
```
