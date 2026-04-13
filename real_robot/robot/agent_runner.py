"""
Decentralised agent runner — runs on each myAGV.

Requires myagv_ros to be running locally (roslaunch myagv_ros ...).

python real_robot/robot/agent_runner.py \
    --config /path/to/network.yaml \
    --id 0 \
    --neighbors 1 2 \
    --goal 5.0 0.0 0.0

TEAM: set self.controller to your swarmlib controller (decentralised variant).
"""
import argparse
import time

import yaml
import zmq
import numpy as np

from real_robot.transport.messages import state_msg, force_msg, unpack
from real_robot.robot.ros1_bridge import ROS1Bridge
from real_robot.robot.load_cell_reader import LoadCellReader
from swarmlib.communication.zmq_backend import ZeroMQSingleAgentBackend


class AgentRunner:
    def __init__(self, robot_id: int, neighbor_ids: list,
                 network_config: dict, goal: np.ndarray,
                 control_hz: float = 20.0):
        self._id = robot_id
        self._neighbors = neighbor_ids
        self._goal = goal
        self._dt = 1.0 / control_hz

        cfg = network_config
        ctx = zmq.Context.instance()

        # PUB: broadcast our state + force to laptop and peers
        my_port = next(r["pub_port"] for r in cfg["robots"] if r["id"] == robot_id)
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{my_port}")

        # SUB: mocap poses from laptop (+ cmd_vel if centralised)
        self._sub = ctx.socket(zmq.SUB)
        self._sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
        self._sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['central_pub_port']}")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "pose")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "cmd")

        # Also subscribe to neighbor state/force for decentralised controller
        for nid in neighbor_ids:
            nip = next(r["ip"] for r in cfg["robots"] if r["id"] == nid)
            nport = next(r["pub_port"] for r in cfg["robots"] if r["id"] == nid)
            self._sub.connect(f"tcp://{nip}:{nport}")
            self._sub.setsockopt_string(zmq.SUBSCRIBE, "state")
            self._sub.setsockopt_string(zmq.SUBSCRIBE, "force")

        # GBP / distributed algorithm peer comms
        self.backend = ZeroMQSingleAgentBackend(
            my_id=robot_id,
            neighbors=neighbor_ids,
            network_config=cfg,
        )

        # Local ROS1 bridge (odom + cmd_vel)
        self._ros = ROS1Bridge(node_name=f"swarm_agent_{robot_id}")

        # Load cells
        self._lc = LoadCellReader()
        self._lc.tare()

        # State buffers
        self._poses = {}          # robot_id → {x, y, theta}
        self._payload_state = np.zeros(6)

        # TEAM: replace None with your decentralised controller
        # e.g.: from swarmlib.controllers import DrCapController
        #       self.controller = DrCapController(my_id=robot_id, ...)
        self.controller = None

        time.sleep(0.2)

    def run(self):
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)
        next_tick = time.monotonic()

        while True:
            # Drain incoming messages
            while dict(poller.poll(timeout=0)):
                topic_bytes, raw = self._sub.recv_multipart()
                d = unpack(raw)
                t = d.get("t")
                if t == "pose":
                    self._poses[d["id"]] = d
                elif t == "cmd" and d.get("id") == self._id:
                    # Centralised command — apply immediately
                    self._ros.send_cmd(d["vx"], d["vy"])

            self._ros.spin_once()

            now = time.monotonic()
            if now >= next_tick:
                odom = self._ros.get_odom()
                lc_readings = self._lc.read()

                # Broadcast own state and force to peers/laptop
                raw_state = state_msg(
                    self._id,
                    odom["x"], odom["y"],
                    odom["vx"], odom["vy"],
                    odom["theta"], odom["omega"],
                )
                self._pub.send_multipart([b"state", raw_state])

                raw_force = force_msg(self._id, lc_readings)
                self._pub.send_multipart([b"force", raw_force])

                # Decentralised control
                if self.controller is not None:
                    robot_states = np.array([[
                        odom["x"], odom["y"], odom["vx"], odom["vy"]
                    ]])
                    # TEAM: pass peer states from self._poses as needed by controller
                    controls = self.controller.compute_control(
                        payload_state=self._payload_state,
                        robot_states=robot_states,
                        goal_state=self._goal,
                        dt=self._dt,
                        forces=None,  # expand once load cell format is finalised
                    )
                    self._ros.send_cmd(float(controls[0, 0]), float(controls[0, 1]))

                next_tick += self._dt

            time.sleep(max(0.0, next_tick - time.monotonic()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/home/ubuntu/network.yaml")
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--neighbors", type=int, nargs="*", default=[])
    parser.add_argument("--goal", type=float, nargs=3, default=[5.0, 0.0, 0.0])
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    runner = AgentRunner(args.id, args.neighbors, cfg, np.array(args.goal))
    runner.run()


if __name__ == "__main__":
    main()
