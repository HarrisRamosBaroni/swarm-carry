# Research Scenario

> This document describes the target research scenario as currently understood.
> It is a living document — assumptions are expected to change as the project develops.

---

## Problem

A team of *n* mobile robots cooperatively transports a rigid payload from a
start configuration to a goal configuration. The environment may contain
obstacles, so the path to the goal is not necessarily straight. Robots must
coordinate to move the payload without dropping it or colliding with obstacles.

---

## Physical setup

**Robots:** Mecanum-wheeled mobile robots (holonomic). Exact platform TBD.
Holonomic drive means each robot can move in any direction independently,
which matters for maintaining formation while navigating around obstacles.

**Payload:** Rigid body, resting on top of the robots (not pushed from the
side). Robots form a platform underneath it. The payload stays on the robots
through friction; shear forces at the contact surfaces are the relevant
sensing modality.

**Sensors:**
- Shear force sensors on the top surface of each robot (robot-payload contact)
- Robot positions assumed known throughout (e.g. from onboard odometry or
  external localisation). This assumption may be revisited.

**Initial conditions:** Initial robot formation and payload pose assumed known.

---

## What is NOT the scenario

The MPC scaling experiment (`experiments/mpc_scaling/`) was a deliberately
simplified checkpoint task: TurtleBot3 diff-drive robots pushing a payload
from one side in a straight line, kinematic model, no obstacles, no force
sensing. It served to measure MPC solve-time scaling in *n* and produce
preliminary results. It is not representative of the real scenario above.

---

## Open questions / things subject to change

- Exact robot platform (currently modelled as Summit XL Steel in the mecanum
  demo, but not finalised)
- Whether robot positions remain fully known throughout, or whether
  localisation uncertainty needs to be modelled
- Obstacle representation (static only, or dynamic)
- Payload shape and mass distribution
- Whether formation is fixed or allowed to reconfigure en route
- Sensor noise models for shear sensors
- Whether inter-robot communication is assumed reliable or subject to dropout

---

## Implications for simulation infrastructure

The current `SwarmTransportEnv` (`swarmlib/simulation/env.py`) does not
support this scenario — it assumes TurtleBot3 diff-drive robots pushing from
the side. When implementing the real scenario, the simulation layer will need:

- A mecanum kinematic model (4 wheel velocities, holonomic)
- Payload-on-top contact geometry in the MuJoCo scene
- Shear force readout from robot-top contact surfaces
- Obstacle bodies in the scene with corresponding state observations
- Robot body/actuator names decoupled from the env (currently hardcoded)

The recommended path is to write a new env (or subclass the existing one) once
the real scene XML and sensor requirements are settled, rather than adapting
the current env prematurely.
