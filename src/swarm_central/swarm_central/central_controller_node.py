"""
CentralControllerNode: centralized multi-robot controller as a ROS2 node.

Runs on the operator laptop (or any machine with network access to the robots).
Aggregates all robot states, runs the controller, publishes cmd_vel to all robots.

Subscribes (from swarm_mujoco_bridge OR real robots):
    /swarm/payload/state          Float64MultiArray [x,y,theta,vx,vy,omega]
    /swarm/robot_{i}/state        Float64MultiArray [x,y,vx,vy]
    /swarm/robot_{i}/force        Float64MultiArray [fx,fy,torque_z]

Publishes:
    /swarm/robot_{i}/cmd_vel      Float64MultiArray [vx,vy]

Team usage
----------
1. Implement (or import) a controller that extends BaseController.
2. Replace `self.controller = None` with your instance.
3. Launch with: ros2 launch swarm_central central.launch.py n_robots:=4

Parameters
----------
n_robots          : int   — number of robots
control_frequency : float — Hz, default 50.0
namespace         : str   — default '/swarm'
goal_x / goal_y / goal_theta : float
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
except ImportError as exc:
    raise ImportError("rclpy / std_msgs not found. Source a ROS2 workspace first.") from exc

# Locate control_scaling_experiments
_pkg_src = Path(__file__).parent.parent.parent.parent  # src/
if str(_pkg_src) not in sys.path:
    sys.path.insert(0, str(_pkg_src))


class CentralControllerNode(Node):
    """
    Centralized controller that subscribes to all robot states and
    publishes cmd_vel to all robots.

    TEAM: replace `self.controller = None` with your controller, e.g.:
        from control_scaling_experiments.controllers import CentralizedMPC
        self.controller = CentralizedMPC(num_robots=n_robots, config={...})
    """

    def __init__(self):
        super().__init__('swarm_central')

        self.declare_parameter('n_robots', 2)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('namespace', '/swarm')
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_theta', 0.0)

        n_robots: int = self.get_parameter('n_robots').value
        ctrl_freq: float = float(self.get_parameter('control_frequency').value)
        ns: str = self.get_parameter('namespace').value.rstrip('/')

        self._n_robots = n_robots
        self._dt = 1.0 / ctrl_freq
        self._goal = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_theta').value,
        ])

        # State buffers
        self._payload_state: np.ndarray = np.zeros(6)
        self._robot_states: np.ndarray = np.zeros((n_robots, 4))
        self._forces: np.ndarray = np.zeros((n_robots, 3))

        # ----------------------------------------------------------------
        # TEAM: replace None with your controller
        # e.g.: self.controller = CentralizedMPC(num_robots=n_robots, config={...})
        # ----------------------------------------------------------------
        self.controller = None

        # Payload subscription
        self.create_subscription(
            Float64MultiArray,
            f'{ns}/payload/state',
            self._payload_cb,
            10,
        )

        # Per-robot subscriptions and publishers
        self._cmd_pubs: List = []
        for i in range(n_robots):
            self.create_subscription(
                Float64MultiArray,
                f'{ns}/robot_{i}/state',
                self._make_state_cb(i),
                10,
            )
            self.create_subscription(
                Float64MultiArray,
                f'{ns}/robot_{i}/force',
                self._make_force_cb(i),
                10,
            )
            self._cmd_pubs.append(
                self.create_publisher(
                    Float64MultiArray,
                    f'{ns}/robot_{i}/cmd_vel',
                    10,
                )
            )

        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'CentralControllerNode started: n_robots={n_robots}, '
            f'freq={ctrl_freq:.0f}Hz'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _payload_cb(self, msg: Float64MultiArray):
        self._payload_state = np.array(msg.data, dtype=float)

    def _make_state_cb(self, idx: int):
        def cb(msg: Float64MultiArray):
            self._robot_states[idx] = np.array(msg.data, dtype=float)
        return cb

    def _make_force_cb(self, idx: int):
        def cb(msg: Float64MultiArray):
            self._forces[idx] = np.array(msg.data, dtype=float)
        return cb

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        if self.controller is None:
            for pub in self._cmd_pubs:
                msg = Float64MultiArray()
                msg.data = [0.0, 0.0]
                pub.publish(msg)
            return

        controls = self.controller.compute_control(
            payload_state=self._payload_state,
            robot_states=self._robot_states,
            goal_state=self._goal,
            dt=self._dt,
            forces=self._forces,
        )

        for i, pub in enumerate(self._cmd_pubs):
            msg = Float64MultiArray()
            msg.data = [float(controls[i, 0]), float(controls[i, 1])]
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CentralControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
