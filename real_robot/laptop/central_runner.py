"""
Centralised controller runner — laptop only.

python real_robot/laptop/central_runner.py \
    --config real_robot/config/network.yaml \
    --n-robots 2 \
    --goal 5.0 0.0 0.0

TEAM: set self.controller to your swarmlib controller before running.
"""
import argparse
import time

import yaml
import zmq
import numpy as np

from real_robot.transport.messages import cmd_msg, unpack


class CentralRunner:
    def __init__(self, network_config: dict, n_robots: int,
                 goal: np.ndarray, control_hz: float = 20.0):
        self._n = n_robots
        self._goal = goal
        self._dt = 1.0 / control_hz

        cfg = network_config
        ctx = zmq.Context.instance()

        # SUB: receive state + force from all robots
        self._sub = ctx.socket(zmq.SUB)
        for r in cfg["robots"][:n_robots]:
            self._sub.connect(f"tcp://{r['ip']}:{r['pub_port']}")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "state")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "force")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "pose")

        # PUB: send cmd_vel to robots
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{cfg['laptop']['central_pub_port']}")

        self._robot_states = np.zeros((n_robots, 4))   # [x,y,vx,vy]
        self._forces = np.zeros((n_robots, 3))          # placeholder shape
        self._payload_state = np.zeros(6)               # from mocap

        # TEAM: replace None with your controller
        # e.g.: from swarmlib.controllers import CentralizedMPC
        #       self.controller = CentralizedMPC(num_robots=n_robots, ...)
        self.controller = None

        time.sleep(0.2)  # allow ZeroMQ connections to establish

    def run(self):
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)
        next_tick = time.monotonic()

        while True:
            # Drain incoming state/force messages
            while dict(poller.poll(timeout=0)):
                topic, raw = self._sub.recv_multipart()
                d = unpack(raw)
                t = d.get("t")
                rid = d.get("id", 0)
                if t == "state" and rid < self._n:
                    self._robot_states[rid] = [d["x"], d["y"], d["vx"], d["vy"]]
                elif t == "force" and rid < self._n:
                    # Adapt when load cell format is finalised
                    pass
                elif t == "pose":
                    # Use payload rigid body pose for payload_state if needed
                    pass

            # Control tick
            now = time.monotonic()
            if now >= next_tick:
                if self.controller is not None:
                    controls = self.controller.compute_control(
                        payload_state=self._payload_state,
                        robot_states=self._robot_states,
                        goal_state=self._goal,
                        dt=self._dt,
                        forces=self._forces,
                    )
                else:
                    controls = np.zeros((self._n, 2))

                for i in range(self._n):
                    raw = cmd_msg(i, float(controls[i, 0]), float(controls[i, 1]))
                    self._pub.send_multipart([b"cmd", raw])

                next_tick += self._dt

            time.sleep(max(0.0, next_tick - time.monotonic()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="real_robot/config/network.yaml")
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--goal", type=float, nargs=3, default=[5.0, 0.0, 0.0])
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    runner = CentralRunner(cfg, args.n_robots, np.array(args.goal))
    runner.run()


if __name__ == "__main__":
    main()
