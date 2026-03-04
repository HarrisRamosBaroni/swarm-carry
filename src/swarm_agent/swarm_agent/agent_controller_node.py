"""
AgentControllerNode: per-robot ROS2 controller node.

Runs on each physical robot (or as a separate process for each simulated agent).
Subscribes to state/force topics published by swarm_mujoco_bridge (or real hardware),
computes a control command, and publishes it back.

Team usage
----------
1. Implement a controller that extends BaseController.
2. Replace `self.controller = None` with your controller instance.
3. Launch one node per robot with a unique agent_id.

Parameters (set via config/agent_params.yaml or launch args)
-------------------------------------------------------------
agent_id          : int    — this robot's ID
neighbor_ids      : string — comma-separated neighbor IDs, e.g. "0,2"
control_frequency : float  — Hz, default 50.0
namespace         : string — topic namespace, default '/swarm'
goal_x            : float  — goal x position
goal_y            : float  — goal y position
goal_theta        : float  — goal orientation
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
except ImportError as exc:
    raise ImportError(
        "rclpy / std_msgs not found. Source a ROS2 workspace first."
    ) from exc

# Locate control_scaling_experiments package
_pkg_src = Path(__file__).parent.parent.parent.parent  # src/
if str(_pkg_src) not in sys.path:
    sys.path.insert(0, str(_pkg_src))

from control_scaling_experiments.communication.ros2_backend import SingleAgentROS2Backend  # noqa


class AgentControllerNode(Node):
    """
    Per-robot controller node.

    Subscribes to:
        /swarm/robot_{agent_id}/state   Float64MultiArray [x,y,vx,vy]
        /swarm/robot_{agent_id}/force   Float64MultiArray [fx,fy,torque_z]
        /swarm/payload/state            Float64MultiArray [x,y,theta,vx,vy,omega]

    Publishes to:
        /swarm/robot_{agent_id}/cmd_vel Float64MultiArray [vx,vy]

    TEAM: replace `self.controller = None` with your controller instance.
    """

    def __init__(self):
        super().__init__('swarm_agent')

        # Parameters
        self.declare_parameter('agent_id', 0)
        self.declare_parameter('neighbor_ids', '')
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('namespace', '/swarm')
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_theta', 0.0)

        self._agent_id: int = self.get_parameter('agent_id').value
        neighbor_str: str = self.get_parameter('neighbor_ids').value
        self._neighbors: List[int] = (
            [int(x.strip()) for x in neighbor_str.split(',') if x.strip()]
            if neighbor_str else []
        )
        ctrl_freq: float = float(self.get_parameter('control_frequency').value)
        ns: str = self.get_parameter('namespace').value.rstrip('/')

        self._goal = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_theta').value,
        ])
        self._dt: float = 1.0 / ctrl_freq

        # State buffers (filled by subscriptions)
        self._robot_state: np.ndarray = np.zeros(4)   # [x,y,vx,vy]
        self._force: np.ndarray = np.zeros(3)          # [fx,fy,torque_z]
        self._payload_state: np.ndarray = np.zeros(6)  # [x,y,theta,vx,vy,omega]

        # Communication backend (for GBP or other distributed algorithms)
        # Replace with SimulatedBackend for pure-Python testing.
        self.backend = SingleAgentROS2Backend(
            my_id=self._agent_id,
            neighbors=self._neighbors,
            namespace=ns,
        )

        # ----------------------------------------------------------------
        # TEAM: replace None with your controller
        # e.g.: from my_pkg.my_controller import MyController
        #        self.controller = MyController(num_robots=total_n, ...)
        # ----------------------------------------------------------------
        self.controller = None

        # Subscriptions
        self.create_subscription(
            Float64MultiArray,
            f'{ns}/robot_{self._agent_id}/state',
            self._state_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            f'{ns}/robot_{self._agent_id}/force',
            self._force_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            f'{ns}/payload/state',
            self._payload_cb,
            10,
        )

        # Publisher
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            f'{ns}/robot_{self._agent_id}/cmd_vel',
            10,
        )

        # Control loop timer
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'AgentControllerNode started: id={self._agent_id}, '
            f'neighbors={self._neighbors}, freq={ctrl_freq:.0f}Hz'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _state_cb(self, msg: Float64MultiArray):
        self._robot_state = np.array(msg.data, dtype=float)

    def _force_cb(self, msg: Float64MultiArray):
        self._force = np.array(msg.data, dtype=float)

    def _payload_cb(self, msg: Float64MultiArray):
        self._payload_state = np.array(msg.data, dtype=float)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        if self.controller is None:
            # No controller set — publish zero command
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0]
            self._cmd_pub.publish(msg)
            return

        # NOTE: robot_states here only contains this robot's state.
        # For centralized algorithms use swarm_central instead.
        robot_states = self._robot_state[np.newaxis, :]  # (1, 4)
        forces = self._force[np.newaxis, :]              # (1, 3)

        controls = self.controller.compute_control(
            payload_state=self._payload_state,
            robot_states=robot_states,
            goal_state=self._goal,
            dt=self._dt,
            forces=forces,
        )

        # Publish this robot's cmd_vel [vx, vy]
        cmd = controls[0]  # (2,)
        msg = Float64MultiArray()
        msg.data = [float(cmd[0]), float(cmd[1])]
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AgentControllerNode()
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
