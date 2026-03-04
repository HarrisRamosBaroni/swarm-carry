#!/usr/bin/env python3
"""
Mecanum Robot ROS2 Bridge Demo
================================
Loads the Summit XL Steel mecanum robot in MuJoCo and exposes its velocity
interface over ROS2 topics — the same pattern as the swarm_mujoco_bridge,
but for a single holonomic robot. Useful for showing supervisors that:

  1. MuJoCo runs in real-time with a realistic mecanum-wheel contact model
  2. The robot is fully commanded via ROS2 topics
  3. The same topic interface works for both simulation and real hardware

Run
---
  # Terminal 1 — start simulation (source ROS2 first, then activate venv)
  source /opt/ros/jazzy/setup.bash
  source ../mj-venv/bin/activate
  python demo.py

  # Terminal 2 — send commands
  # Forward (vx = 0.3 m/s):
  ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.3, 0.0, 0.0]"

  # Strafe right (vy = -0.3 m/s):
  ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.0, -0.3, 0.0]"

  # Rotate CCW (omega = 0.5 rad/s):
  ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.5]"

  # Stop:
  ros2 topic pub --once /mecanum/cmd_vel std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0]"

  # Watch state:
  ros2 topic echo /mecanum/odom

Topics
------
  Sub: /mecanum/cmd_vel   std_msgs/Float64MultiArray  [vx (m/s), vy (m/s), omega (rad/s)]
  Pub: /mecanum/odom      std_msgs/Float64MultiArray  [x, y, theta, vx, vy, omega]

Mecanum kinematics
------------------
  Summit XL Steel: wheel_radius=0.12 m, lx=0.2225 m, ly=0.2045 m
  Inverse kinematics (body vel → wheel rad/s):
    fl =  (vx - vy - (lx+ly)*omega) / r
    fr =  (vx + vy + (lx+ly)*omega) / r
    bl =  (vx + vy - (lx+ly)*omega) / r
    br =  (vx - vy + (lx+ly)*omega) / r
"""

from __future__ import annotations

import sys
import time
import threading
from pathlib import Path

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    sys.exit("MuJoCo not found. Activate the mj-venv and try again.")

def _find_ros2_python_paths():
    """Add ROS2 dist-packages to sys.path if AMENT_PREFIX_PATH is set."""
    import os, glob
    prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    if not prefix_path:
        return
    py_tag = f"python{sys.version_info.major}.{sys.version_info.minor}"
    for prefix in prefix_path.split(':'):
        candidate = os.path.join(prefix, 'lib', py_tag, 'dist-packages')
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)

_ROS2 = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    _ROS2 = True
except ImportError:
    _find_ros2_python_paths()
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray
        _ROS2 = True
    except ImportError:
        print("[WARN] rclpy not found — running without ROS2 (no topics).")
        print("       Source a ROS2 workspace before running.")

# Stub so class definition below doesn't fail when ROS2 is unavailable
if not _ROS2:
    class Node:  # noqa: F811
        def __init__(self, *a, **kw): pass

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_REPO_ROOT = Path(__file__).parent.parent
_SCENE_XML = _REPO_ROOT / "holonomic_dp" / "robots" / "summit_xl_description" / "summit_xls.xml"

if not _SCENE_XML.exists():
    sys.exit(
        f"Scene XML not found: {_SCENE_XML}\n"
        "Run: git submodule update --init src/holonomic_dp"
    )

# ---------------------------------------------------------------------------
# Mecanum kinematics
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.120  # metres (Summit XL Steel mecanum wheel)
LX = 0.2225           # half wheelbase (front/back half distance)
LY = 0.2045           # half track width (left/right half distance)
L = LX + LY           # geometric factor for omega contribution

# Proportional gain for wheel velocity tracking (same as summit_test.py)
WHEEL_KV = 200.0


def body_to_wheel(vx: float, vy: float, omega: float) -> np.ndarray:
    """
    Mecanum inverse kinematics: body velocity [vx, vy, omega] → wheel angular
    velocities [fl, fr, bl, br] in rad/s.

    Convention (robot frame):
        vx > 0 = forward,  vy > 0 = left,  omega > 0 = CCW
    """
    r = WHEEL_RADIUS
    fl = ( vx - vy - L * omega) / r
    fr = ( vx + vy + L * omega) / r
    bl = ( vx + vy - L * omega) / r
    br = ( vx - vy + L * omega) / r
    return np.array([fl, fr, bl, br])


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class MecanumBridgeNode(Node):
    """
    ROS2 interface for the mecanum MuJoCo demo.

    Sub: /mecanum/cmd_vel  — velocity setpoint [vx, vy, omega]
    Pub: /mecanum/odom     — body state [x, y, theta, vx, vy, omega]  @ 50 Hz
    """

    def __init__(self, shared: dict):
        super().__init__('mecanum_bridge')
        self._shared = shared

        self.create_subscription(
            Float64MultiArray,
            '/mecanum/cmd_vel',
            self._cmd_cb,
            10,
        )
        self._odom_pub = self.create_publisher(
            Float64MultiArray, '/mecanum/odom', 10
        )
        # Publish odom at 50 Hz from ROS2 timer (reads from shared state)
        self.create_timer(0.02, self._publish_odom)

        self.get_logger().info(
            "MecanumBridgeNode started.\n"
            "  Sub: /mecanum/cmd_vel  [vx, vy, omega]\n"
            "  Pub: /mecanum/odom     [x, y, theta, vx, vy, omega]"
        )

    def _cmd_cb(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 3:
            self.get_logger().warn(f"cmd_vel needs 3 values, got {len(data)}")
            return
        vx, vy, omega = float(data[0]), float(data[1]), float(data[2])
        self._shared['target_wheel_vel'] = body_to_wheel(vx, vy, omega)
        self.get_logger().info(
            f"cmd_vel: vx={vx:.2f} vy={vy:.2f} omega={omega:.2f}  →  "
            f"wheels(fl,fr,bl,br)={self._shared['target_wheel_vel'].round(2)}"
        )

    def _publish_odom(self):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self._shared['robot_state']]
        self._odom_pub.publish(msg)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # Shared state dict between MuJoCo main thread and ROS2 spin thread
    shared = {
        'target_wheel_vel': np.zeros(4),   # [fl, fr, bl, br] rad/s targets
        'robot_state': np.zeros(6),         # [x, y, theta, vx, vy, omega]
    }

    # ------------------------------------------------------------------
    # Load MuJoCo model
    # ------------------------------------------------------------------
    print(f"Loading model: {_SCENE_XML}")
    model = mujoco.MjModel.from_xml_path(str(_SCENE_XML))
    data = mujoco.MjData(model)

    # Joint qvel DOF addresses for the four rolling joints
    # (programmatic lookup so we don't hard-code qvel indices)
    joint_names = [
        'front_left_wheel_rolling_joint',
        'front_right_wheel_rolling_joint',
        'back_left_wheel_rolling_joint',
        'back_right_wheel_rolling_joint',
    ]
    qvel_ids = []
    for jname in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            sys.exit(f"Joint not found in model: {jname}")
        qvel_ids.append(model.jnt_dofadr[jid])
    fl_qv, fr_qv, bl_qv, br_qv = qvel_ids

    # Actuator indices (order in summit_xls_actuator.xml):
    #   ctrl[0] = front_right, ctrl[1] = front_left,
    #   ctrl[2] = back_right,  ctrl[3] = back_left
    fr_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'front_right_wheel_rolling_joint')
    fl_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'front_left_wheel_rolling_joint')
    br_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'back_right_wheel_rolling_joint')
    bl_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'back_left_wheel_rolling_joint')

    # Base body ID for state extraction
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_footprint')
    if base_id < 0:
        sys.exit("Body 'base_footprint' not found in model.")

    print(f"Model loaded: {model.nbody} bodies, {model.nu} actuators")

    # ------------------------------------------------------------------
    # Start ROS2 in background thread
    # ------------------------------------------------------------------
    ros_thread = None
    node = None
    if _ROS2:
        rclpy.init()
        node = MecanumBridgeNode(shared)
        ros_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True
        )
        ros_thread.start()
        print("ROS2 node started in background thread.")
    else:
        print("ROS2 unavailable — simulation only.")

    # ------------------------------------------------------------------
    # MuJoCo main loop (runs in main thread with viewer)
    # ------------------------------------------------------------------
    print("\nOpening MuJoCo viewer... close the window to exit.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Nice camera angle to see the robot
        viewer.cam.distance = 4.0
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -30.0

        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)

            # -- Read current wheel velocities --
            current_wheel_vel = np.array([
                data.qvel[fl_qv],  # front_left
                data.qvel[fr_qv],  # front_right
                data.qvel[bl_qv],  # back_left
                data.qvel[br_qv],  # back_right
            ])

            # -- PID: torque = kv * (target - current) --
            target = shared['target_wheel_vel']  # [fl, fr, bl, br]
            error = target - current_wheel_vel
            torque = error * WHEEL_KV

            data.ctrl[fl_act] = torque[0]
            data.ctrl[fr_act] = torque[1]
            data.ctrl[bl_act] = torque[2]
            data.ctrl[br_act] = torque[3]

            # -- Extract robot body state for ROS2 publishing --
            pos = data.xpos[base_id]                       # [x, y, z]
            quat = data.xquat[base_id]                     # [qw, qx, qy, qz]
            vel = data.cvel[base_id]                       # [wx,wy,wz, vx,vy,vz]

            qw, qx, qy, qz = quat
            theta = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy**2 + qz**2))
            shared['robot_state'] = np.array([
                pos[0], pos[1], theta,
                vel[3], vel[4], vel[2],  # vx, vy, omega
            ])

            viewer.sync()

            # Real-time pacing
            elapsed = time.time() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    if node is not None:
        node.destroy_node()
    if _ROS2 and rclpy.ok():
        rclpy.shutdown()
    print("Done.")


if __name__ == '__main__':
    main()
