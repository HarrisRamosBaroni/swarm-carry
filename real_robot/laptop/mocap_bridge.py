"""
Mocap bridge — laptop only.

Subscribes to /mocap/rigid_{id} ROS2 topics (from swarm_mocap node).
Rebroadcasts all rigid body poses over ZeroMQ so robots can subscribe.

Run AFTER swarm_mocap is already publishing:
  ros2 launch swarm_mocap mocap.launch.py server_ip:=192.168.0.244

Then:
  python -m real_robot.laptop.mocap_bridge --config real_robot/config/network.yaml
"""
import argparse
import math

import yaml
import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from real_robot.transport.messages import pose_msg


class MocapBridge(Node):
    def __init__(self, network_config: dict, rigid_body_ids: dict):
        super().__init__("mocap_zmq_bridge")

        ctx = zmq.Context.instance()
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{network_config['laptop']['mocap_pub_port']}")

        # Subscribe to each rigid body's per-body topic. robot_<i> maps to
        # id=i; the optional "payload" key maps to id=-1 (sentinel — robots
        # never have negative ids) so central_runner can pick it out.
        for key, rb_id in rigid_body_ids.items():
            if key.startswith("robot_"):
                rid = int(key.split("_")[1])
            elif key == "payload":
                rid = -1
            else:
                continue
            self.create_subscription(
                PoseStamped,
                f"/mocap/rigid_{rb_id}",
                self._make_cb(rid),
                10,
            )
        self.get_logger().info("MocapBridge running")

    def _make_cb(self, robot_id: int):
        def cb(msg: PoseStamped):
            q = msg.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny, cosy)
            raw = pose_msg(robot_id,
                           msg.pose.position.x,
                           msg.pose.position.y,
                           theta)
            self._pub.send_multipart([b"pose", raw])
        return cb


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="real_robot/config/network.yaml")
    args = parser.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    rclpy.init()
    node = MocapBridge(cfg, cfg["mocap"]["rigid_body_ids"])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
