# Mecanum ROS2 Bridge Demo

Single holonomic (mecanum-wheel) robot in MuJoCo, commanded over ROS2 topics.
Uses the Summit XL Steel model from `src/holonomic_dp` (submodule).

**What it shows:**
- MuJoCo running a realistic mecanum-wheel contact model in real-time
- Robot velocity commanded via `/mecanum/cmd_vel` (the same pattern as the swarm bridge)
- Odometry published to `/mecanum/odom`

---

## Setup (one-time)

```bash
# 1. Initialize the submodule if not done yet
git submodule update --init src/holonomic_dp

# 2. Confirm mujoco is available
/path/to/mj-venv/bin/python -c "import mujoco; print(mujoco.__version__)"
```

---

## Run

**Terminal 1 — launch the simulation:**
```bash
source /opt/ros/humble/setup.bash
cd src/mecanum_ros2_demo
/path/to/mj-venv/bin/python demo.py
```

A MuJoCo viewer opens showing the Summit XL Steel robot.

**Terminal 2 — send velocity commands:**
```bash
source /opt/ros/humble/setup.bash

# Forward (vx = 0.3 m/s)
ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.3, 0.0, 0.0]"

# Strafe right (vy = -0.3 m/s, note ROS convention: +y is left)
ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.0, -0.3, 0.0]"

# Strafe left (vy = +0.3 m/s)
ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.0, 0.3, 0.0]"

# Rotate CCW (omega = 0.5 rad/s)
ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.5]"

# Diagonal (forward + strafe simultaneously — only possible with holonomic drive)
ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.3, 0.3, 0.0]"

# Stop
ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0]"
```

**Watch odometry:**
```bash
ros2 topic echo /mecanum/odom   # [x, y, theta, vx, vy, omega]
```

---

## Topics

| Topic | Direction | Type | Content |
|-------|-----------|------|---------|
| `/mecanum/cmd_vel` | Subscribe | `Float64MultiArray` | `[vx (m/s), vy (m/s), omega (rad/s)]` |
| `/mecanum/odom` | Publish | `Float64MultiArray` | `[x, y, theta, vx, vy, omega]` |

---

## Mecanum Kinematics

The Summit XL Steel uses 4 independently driven mecanum wheels.
Wheel velocity is commanded via a proportional controller (kv=200).

```
wheel_radius = 0.120 m,  lx = 0.2225 m (half-wheelbase),  ly = 0.2045 m (half-track)
L = lx + ly = 0.427 m

fl = ( vx - vy - L·ω) / r      (front-left)
fr = ( vx + vy + L·ω) / r      (front-right)
bl = ( vx + vy - L·ω) / r      (back-left)
br = ( vx - vy + L·ω) / r      (back-right)
```

The contact model uses rolling sphere geoms at 45° angles — this is what allows
sideways force generation without MuJoCo velocity actuators (which can't model
mecanum kinematics correctly).

---

## Architecture

```
demo.py
├── Main thread: MuJoCo step loop + viewer (real-time paced)
│     reads shared['target_wheel_vel']
│     writes shared['robot_state']
└── Background thread: rclpy.spin(node)
      MecanumBridgeNode:
        /mecanum/cmd_vel → shared['target_wheel_vel']  (on receipt)
        shared['robot_state'] → /mecanum/odom           (50 Hz timer)
```

This is structurally identical to `swarm_mujoco_bridge`, but for a single
holonomic robot. The same pattern scales to the full swarm.
