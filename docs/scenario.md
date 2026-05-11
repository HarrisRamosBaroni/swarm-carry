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

**Robots:** Mecanum-wheeled mobile robots (holonomic). ~~Exact platform TBD.~~ myAGV (pre 2023).
Holonomic drive means each robot can move in any direction independently,
which matters for maintaining formation while navigating around obstacles.

**Payload:** ~~Rigid body, resting on top of the robots (not pushed from the
side). Robots form a platform underneath it. The payload stays on the robots
through friction; shear forces at the contact surfaces are the relevant
sensing modality.~~ Rigid body with flat faces (box). Each robot grips a face
using a forklift-style holder fixed to the robot's front. The holder has a
horizontal fork base (extends under the payload) and a vertical fork wall
(presses against the payload face). The payload is supported from below by
the fork bases and laterally constrained by the fork walls.

**Sensors:**
- ~~Shear force sensors on the top surface of each robot (robot-payload contact)~~
- Force sensor on the fork base bottom plane (vertical load — weight bearing)
- Force sensor on the fork wall vertical plane (horizontal force into the robot)
- Robot positions assumed known throughout (~~e.g. from onboard odometry or~~ from
  external localisation). ~~This assumption may be revisited.~~
- Payload position assumed known throughout.

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

- ~~Exact robot platform (currently modelled as Summit XL Steel in the mecanum
  demo, but not finalised)~~ myAGV pre 2023
- ~~Whether robot positions remain fully known throughout, or whether
  localisation uncertainty needs to be modelled~~ robot positions known.
- ~~Obstacle representation (static only, or dynamic)~~ static or none
- ~~Payload shape and mass distribution~~ cuboidal, roughly uniform
- ~~Whether formation is fixed or allowed to reconfigure en route~~ fixed target formation
- ~~Sensor noise models for shear sensors~~ not applicable
- ~~Whether inter-robot communication is assumed reliable or subject to dropout~~ this is a robustness test