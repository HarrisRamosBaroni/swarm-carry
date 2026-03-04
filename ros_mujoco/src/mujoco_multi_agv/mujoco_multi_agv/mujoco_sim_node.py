# mujoco_multi_agv/mujoco_sim_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from .mujoco_interface import MujocoInterface

from ament_index_python.packages import get_package_share_directory
import os


WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.288


class MujocoSimNode(Node):

    def __init__(self):
        super().__init__('mujoco_sim_node')

        self.declare_parameter("scene_path", "")
        self.declare_parameter("num_robots", 2)

        self.num_robots = self.get_parameter("num_robots").value
        scene_path = self.get_parameter("scene_path").value

        # If no scene_path provided, load default from package
        if scene_path == "":
            package_share = get_package_share_directory("mujoco_multi_agv")
            scene_path = os.path.join(package_share, "scenes", "scene.xml")

            self.get_logger().info(f"Loading default scene: {scene_path}")
        else:
            self.get_logger().info(f"Loading scene from parameter: {scene_path}")

        self.sim = MujocoInterface(scene_path, self.num_robots)

        self.cmd_vel = {
            i: (0.0, 0.0) for i in range(self.num_robots)
        }

        self.odom_pubs = {}
        self.joint_pubs = {}
        self.tf_broadcasters = {}

        for i in range(self.num_robots):
            ns = f"/robot_{i}"

            self.create_subscription(
                Twist,
                f"{ns}/cmd_vel",
                lambda msg, robot_id=i: self.cmd_callback(msg, robot_id),
                10
            )

            self.odom_pubs[i] = self.create_publisher(
                Odometry, f"{ns}/odom", 10
            )

            self.joint_pubs[i] = self.create_publisher(
                JointState, f"{ns}/joint_states", 10
            )

            self.tf_broadcasters[i] = TransformBroadcaster(self)

        self.timer = self.create_timer(0.005, self.sim_step)

        self.get_logger().info("MuJoCo ROS2 Simulation Started")

    # --------------------------
    # Callbacks
    # --------------------------

    def cmd_callback(self, msg, robot_id):
        v = msg.linear.x
        w = msg.angular.z

        left = (v - w * WHEEL_BASE / 2) / WHEEL_RADIUS
        right = (v + w * WHEEL_BASE / 2) / WHEEL_RADIUS

        self.cmd_vel[robot_id] = (left, right)

    # --------------------------
    # Simulation Loop
    # --------------------------

    def sim_step(self):

        # Apply commands
        for i in range(self.num_robots):
            left, right = self.cmd_vel[i]
            self.sim.set_wheel_velocity(i, left, right)

        # Step physics
        self.sim.step()

        # Publish state
        for i in range(self.num_robots):
            self.publish_robot_state(i)

    # --------------------------
    # State Publishing
    # --------------------------

    def publish_robot_state(self, robot_id):

        pos, quat = self.sim.get_robot_pose(robot_id)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "world"
        odom.child_frame_id = f"robot_{robot_id}/base_link"

        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]

        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.pose.orientation.w = quat[0]

        self.odom_pubs[robot_id].publish(odom)

        # TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcasters[robot_id].sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()