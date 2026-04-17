"""
HolonomicBridgeNode: MuJoCo physics (holonomic + L-carriage) as a ROS2 node.

Drop-in replacement for SwarmMujocoBridgeNode but using HolonomicTransportEnv.

Published topics
----------------
/swarm/payload/state                Float64MultiArray  [x,y,theta,vx,vy,omega]
/swarm/robot_{i}/state              Float64MultiArray  [x,y,vx,vy]
/swarm/robot_{i}/force              Float64MultiArray  [fx,fy,fz]   ← base force
                                    (backward-compatible with controller nodes)
/swarm/robot_{i}/carriage_base_force Float64MultiArray [fx,fy,fz] site frame
/swarm/robot_{i}/carriage_wall_force Float64MultiArray [fx,fy,fz] site frame

Subscribed topics
-----------------
/swarm/robot_{i}/cmd_vel            Float64MultiArray  [vx,vy]  world frame

Parameters
----------
n_robots          : int   (default 2)
scene_xml         : str   (default '', auto-generate)
sim_frequency     : float (default 200.0 Hz)
control_frequency : float (default 50.0 Hz)
goal_x/y/theta    : float
payload_mass      : float (default 10.0 kg)
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
        "rclpy / std_msgs not found. Source a ROS2 workspace first."
    ) from exc

from swarmlib.simulation import HolonomicTransportEnv


class HolonomicBridgeNode(Node):
    """ROS2 node wrapping HolonomicTransportEnv as a pub/sub physics bridge."""

    def __init__(self):
        super().__init__('holonomic_mujoco_bridge')

        self.declare_parameter('n_robots', 2)
        self.declare_parameter('scene_xml', '')
        self.declare_parameter('sim_frequency', 200.0)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_theta', 0.0)
        self.declare_parameter('payload_mass', 10.0)

        n_robots = self.get_parameter('n_robots').value
        scene_xml = self.get_parameter('scene_xml').value or None
        sim_freq = float(self.get_parameter('sim_frequency').value)
        ctrl_freq = float(self.get_parameter('control_frequency').value)
        goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            float(self.get_parameter('goal_theta').value),
        )
        payload_mass = float(self.get_parameter('payload_mass').value)

        self._n_robots = n_robots
        self._ctrl_dt = 1.0 / ctrl_freq
        self._steps_per_ctrl = max(1, round(sim_freq / ctrl_freq))

        self._env = HolonomicTransportEnv(
            n_robots=n_robots,
            scene_xml=scene_xml,
            goal=goal,
            payload_mass=payload_mass,
            dt_control=1.0 / sim_freq,  # single-substep per env.step call
        )
        self._obs = self._env.reset()
        self._cmd_vel: List[np.ndarray] = [np.zeros(2) for _ in range(n_robots)]

        # Publishers
        self._pub_payload = self.create_publisher(
            Float64MultiArray, '/swarm/payload/state', 10
        )
        self._pub_state: List = []
        self._pub_force: List = []           # backward-compat: base force
        self._pub_base_force: List = []      # carriage base force
        self._pub_wall_force: List = []      # carriage wall force

        for i in range(n_robots):
            self._pub_state.append(
                self.create_publisher(Float64MultiArray, f'/swarm/robot_{i}/state', 10)
            )
            self._pub_force.append(
                self.create_publisher(Float64MultiArray, f'/swarm/robot_{i}/force', 10)
            )
            self._pub_base_force.append(
                self.create_publisher(
                    Float64MultiArray, f'/swarm/robot_{i}/carriage_base_force', 10
                )
            )
            self._pub_wall_force.append(
                self.create_publisher(
                    Float64MultiArray, f'/swarm/robot_{i}/carriage_wall_force', 10
                )
            )

        # Subscriptions
        for i in range(n_robots):
            self.create_subscription(
                Float64MultiArray,
                f'/swarm/robot_{i}/cmd_vel',
                self._make_cmd_cb(i),
                10,
            )

        self.create_timer(self._ctrl_dt, self._timer_cb)

        self.get_logger().info(
            f'HolonomicBridgeNode started: n_robots={n_robots}, '
            f'sim={sim_freq:.0f}Hz ctrl={ctrl_freq:.0f}Hz'
        )

    def _make_cmd_cb(self, idx: int):
        def cb(msg: Float64MultiArray):
            data = np.array(msg.data, dtype=float)
            if data.shape == (2,):
                self._cmd_vel[idx] = data
        return cb

    def _timer_cb(self):
        controls = np.stack(self._cmd_vel)
        for _ in range(self._steps_per_ctrl):
            self._obs = self._env.step(controls)

        # Payload
        payload_msg = Float64MultiArray()
        payload_msg.data = self._obs['payload'].tolist()
        self._pub_payload.publish(payload_msg)

        # Per-robot
        for i in range(self._n_robots):
            state_msg = Float64MultiArray()
            state_msg.data = self._obs['robots'][i].tolist()
            self._pub_state[i].publish(state_msg)

            base_f = self._obs['base_forces'][i]
            wall_f = self._obs['wall_forces'][i]

            # Backward-compat /force topic carries base force
            force_msg = Float64MultiArray()
            force_msg.data = [float(base_f)]
            self._pub_force[i].publish(force_msg)

            base_msg = Float64MultiArray()
            base_msg.data = [float(base_f)]
            self._pub_base_force[i].publish(base_msg)

            wall_msg = Float64MultiArray()
            wall_msg.data = [float(wall_f)]
            self._pub_wall_force[i].publish(wall_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicBridgeNode()
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
