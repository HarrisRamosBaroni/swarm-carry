"""
Mocap bridge — laptop only.

Subscribes to /mocap/rigid_{id} ROS2 topics (from swarm_mocap node).
Rebroadcasts all rigid body poses over ZeroMQ so central_runner can subscribe.

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


PAYLOAD_ID = -1  # sentinel ZMQ id for the payload rigid body


class MocapBridge(Node):
    def __init__(self, network_config: dict):
        super().__init__("mocap_zmq_bridge")

        ctx = zmq.Context.instance()
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{network_config['laptop']['mocap_pub_port']}")

        for r in network_config["robots"]:
            if "mocap_rigid_id" not in r:
                continue
            self._subscribe(r["mocap_rigid_id"], r["id"])

        if "payload" in network_config:
            self._subscribe(network_config["payload"]["mocap_rigid_id"], PAYLOAD_ID)

        self.get_logger().info("MocapBridge running")

    def _subscribe(self, phasespace_id: int, rid: int):
        self.create_subscription(
            PoseStamped,
            f"/mocap/rigid_{phasespace_id}",
            self._make_cb(rid),
            10,
        )

    def _make_cb(self, rid: int):
        def cb(msg: PoseStamped):
            q = msg.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny, cosy)
            raw = pose_msg(rid,
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
    node = MocapBridge(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
