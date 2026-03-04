"""
SwarmMujocoBridgeNode: MuJoCo physics as a ROS2 node.

Publishes robot/payload states and per-robot forces at control_frequency Hz.
Subscribes to per-robot cmd_vel topics (Float64MultiArray [vx, vy]).
Steps the MuJoCo physics at sim_frequency Hz in a timer callback.

Topic structure
---------------
Published:
    /swarm/payload/state          Float64MultiArray  [x,y,theta,vx,vy,omega]
    /swarm/robot_{i}/state        Float64MultiArray  [x,y,vx,vy]
    /swarm/robot_{i}/force        Float64MultiArray  [fx,fy,torque_z]

Subscribed:
    /swarm/robot_{i}/cmd_vel      Float64MultiArray  [vx,vy]

This is the physics interface. Algorithm peer-to-peer communication belongs in
CommunicationBackend (SimulatedBackend / ROS2Backend), which is a separate concern.
"""

from __future__ import annotations

from typing import List

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
except ImportError as exc:
    raise ImportError(
        "rclpy / std_msgs not found. Source a ROS2 workspace before running "
        "the bridge node."
    ) from exc

from swarmlib.simulation import SwarmTransportEnv


class SwarmMujocoBridgeNode(Node):
    """
    ROS2 node wrapping SwarmTransportEnv as a pub/sub physics bridge.

    Parameters (ROS2 node params)
    -----------------------------
    n_robots         : int   (default 2)
    scene_xml        : str   (default '', auto-generate)
    sim_frequency    : float (default 200.0 Hz)
    control_frequency: float (default 50.0 Hz)
    push_distance    : float (default 5.0 m)  — only used when auto-generating scene
    goal_x           : float (default 5.0)
    goal_y           : float (default 0.0)
    goal_theta       : float (default 0.0)
    """

    def __init__(self):
        super().__init__('swarm_mujoco_bridge')

        # Declare and read parameters
        self.declare_parameter('n_robots', 2)
        self.declare_parameter('scene_xml', '')
        self.declare_parameter('sim_frequency', 200.0)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('push_distance', 5.0)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_theta', 0.0)

        n_robots = self.get_parameter('n_robots').value
        scene_xml = self.get_parameter('scene_xml').value or None
        sim_freq = float(self.get_parameter('sim_frequency').value)
        ctrl_freq = float(self.get_parameter('control_frequency').value)
        push_dist = float(self.get_parameter('push_distance').value)
        goal = [
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_theta').value,
        ]

        self._n_robots = n_robots
        self._sim_dt = 1.0 / sim_freq
        self._ctrl_dt = 1.0 / ctrl_freq
        # How many physics steps per control publish cycle
        self._steps_per_ctrl = max(1, round(sim_freq / ctrl_freq))

        # Create the MuJoCo environment
        self._env = SwarmTransportEnv(
            n_robots=n_robots,
            scene_xml=scene_xml,
            goal_pos=goal,
            push_distance=push_dist,
        )
        self._obs = self._env.reset()

        # Latest buffered cmd_vel per robot (zeros = no command yet)
        self._cmd_vel: List[np.ndarray] = [np.zeros(2) for _ in range(n_robots)]

        # Publishers
        self._pub_payload = self.create_publisher(
            Float64MultiArray, '/swarm/payload/state', 10
        )
        self._pub_robot_state: List = []
        self._pub_robot_force: List = []
        for i in range(n_robots):
            self._pub_robot_state.append(
                self.create_publisher(Float64MultiArray, f'/swarm/robot_{i}/state', 10)
            )
            self._pub_robot_force.append(
                self.create_publisher(Float64MultiArray, f'/swarm/robot_{i}/force', 10)
            )

        # Subscriptions — latest-value latch via closure
        for i in range(n_robots):
            self.create_subscription(
                Float64MultiArray,
                f'/swarm/robot_{i}/cmd_vel',
                self._make_cmd_vel_callback(i),
                10,
            )

        # Timer: step physics + publish at ctrl_freq
        self.create_timer(self._ctrl_dt, self._timer_callback)

        self.get_logger().info(
            f'SwarmMujocoBridgeNode started: n_robots={n_robots}, '
            f'sim_freq={sim_freq:.0f}Hz, ctrl_freq={ctrl_freq:.0f}Hz'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _make_cmd_vel_callback(self, robot_idx: int):
        def callback(msg: Float64MultiArray):
            data = np.array(msg.data, dtype=float)
            if data.shape == (2,):
                self._cmd_vel[robot_idx] = data
        return callback

    # ------------------------------------------------------------------
    # Timer callback: step + publish
    # ------------------------------------------------------------------

    def _timer_callback(self):
        # Step physics (multiple sub-steps if sim_freq > ctrl_freq)
        controls = np.stack(self._cmd_vel)  # (n, 2)
        for _ in range(self._steps_per_ctrl):
            self._obs = self._env.step(controls)

        # Publish payload state
        payload_msg = Float64MultiArray()
        payload_msg.data = self._obs['payload'].tolist()
        self._pub_payload.publish(payload_msg)

        # Publish per-robot state and force
        for i in range(self._n_robots):
            state_msg = Float64MultiArray()
            state_msg.data = self._obs['robots'][i].tolist()
            self._pub_robot_state[i].publish(state_msg)

            force_msg = Float64MultiArray()
            force_msg.data = self._obs['forces'][i].tolist()
            self._pub_robot_force[i].publish(force_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmMujocoBridgeNode()
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
