"""
Centralised controller runner — laptop only.

python real_robot/laptop/central_runner.py \
    --config real_robot/config/network.yaml \
    --n-robots 2 \
    --goal 5.0 0.0 0.0

Robot and payload states come entirely from mocap (via mocap_bridge). Robot
velocities are derived by differentiating consecutive mocap poses. Pass
--gt-payload to use the live mocap payload pose every step instead of the
centroid estimator — requires a "payload" entry in network.yaml and
mocap_bridge running.

The controller is constructed on the first tick that has full mocap data.
Robot-to-payload formation offsets are derived from actual mocap poses at
that moment, so no payload geometry parameters are needed.

TEAM: swap MRCapController for any other centralised controller that follows
the BaseController.compute_control(payload, robots, goal, dt, forces) API.
"""
import argparse
import time

import yaml
import zmq
import numpy as np

from real_robot.transport.messages import cmd_msg, unpack
from swarmlib.controllers import MRCapController


PAYLOAD_ID = -1  # sentinel id used by mocap_bridge for the payload rigid body


class CentralRunner:
    def __init__(self, network_config: dict, n_robots: int,
                 goal: np.ndarray,
                 control_hz: float = 20.0,
                 horizon: int = 15,
                 v_max: float = 0.25,
                 use_gt_payload: bool = False,
                 relative_goal: bool = False):
        self._n = n_robots
        self._goal = goal          # absolute if not relative_goal; offset if relative_goal
        self._goal_offset = goal if relative_goal else None  # resolved on first tick
        self._dt = 1.0 / control_hz
        self._use_gt_payload = use_gt_payload

        cfg = network_config
        ctx = zmq.Context.instance()

        self._sub = ctx.socket(zmq.SUB)
        for r in cfg["robots"][:n_robots]:
            self._sub.connect(f"tcp://{r['ip']}:{r['pub_port']}")
        self._sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "force")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "pose")

        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{cfg['laptop']['central_pub_port']}")

        # robot_states columns: [x, y, theta, vx, vy] — all from mocap.
        # vx/vy are derived by differentiating consecutive pose messages.
        self._robot_states = np.zeros((n_robots, 5))
        self._robot_prev_pose = np.full((n_robots, 3), np.nan)  # (x, y, theta) last tick
        self._robot_prev_ts = np.zeros(n_robots)
        self._got_state = np.zeros(n_robots, dtype=bool)
        self._payload_pose = None  # (x, y, theta) from mocap when available
        self._forces = np.zeros((n_robots, 3))

        self.controller = None
        self._controller_cfg = {
            "horizon": horizon,
            "v_max": v_max,
            "estimate_centroid": not use_gt_payload,
        }

        time.sleep(0.2)

    def _drain(self, poller):
        while dict(poller.poll(timeout=0)):
            _, raw = self._sub.recv_multipart()
            d = unpack(raw)
            if d.get("t") != "pose":
                continue
            rid = d.get("id", 0)
            x, y, theta, ts = d["x"], d["y"], d["theta"], d["ts"]
            if rid == PAYLOAD_ID:
                self._payload_pose = (x, y, theta)
            elif 0 <= rid < self._n:
                prev = self._robot_prev_pose[rid]
                dt = ts - self._robot_prev_ts[rid]
                if not np.isnan(prev[0]) and dt > 0:
                    vx = (x - prev[0]) / dt
                    vy = (y - prev[1]) / dt
                else:
                    vx = vy = 0.0
                self._robot_states[rid] = [x, y, theta, vx, vy]
                self._robot_prev_pose[rid] = [x, y, theta]
                self._robot_prev_ts[rid] = ts
                self._got_state[rid] = True

    def _formation_from_poses(self, payload_state: np.ndarray) -> list:
        p_c = payload_state[:2]
        theta = float(payload_state[2])
        c, s = np.cos(theta), np.sin(theta)
        R_inv = np.array([[c, s], [-s, c]])
        r_body = (self._robot_states[:self._n, :2] - p_c) @ R_inv.T
        return [(float(r_body[i, 0]), float(r_body[i, 1]), 0.0) for i in range(self._n)]

    def _build_payload_state(self) -> np.ndarray:
        # MRCapController only consumes payload_state[:3] (centroid pose).
        if self._use_gt_payload:
            if self._payload_pose is None:
                return np.zeros(6)  # caller skips control until pose arrives
            x, y, th = self._payload_pose
            return np.array([x, y, th, 0.0, 0.0, 0.0])
        # Estimator mode: synthesise init pose from robot positions on the
        # first tick. CentroidEstimator.reset uses this once to calibrate r_i;
        # subsequent ticks ignore payload_state in favour of the estimate.
        x = float(self._robot_states[:self._n, 0].mean())
        y = float(self._robot_states[:self._n, 1].mean())
        return np.array([x, y, 0.0, 0.0, 0.0, 0.0])

    def run(self):
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)
        next_tick = time.monotonic()

        while True:
            self._drain(poller)

            now = time.monotonic()
            if now >= next_tick:
                ready = self._got_state.all() and (
                    not self._use_gt_payload or self._payload_pose is not None
                )
                if ready and self._goal_offset is not None:
                    # Resolve relative goal once on the first ready tick.
                    start_x = float(self._robot_states[:self._n, 0].mean())
                    start_y = float(self._robot_states[:self._n, 1].mean())
                    self._goal = self._goal_offset + np.array([start_x, start_y, 0.0])
                    print(f"[central] relative goal resolved to {self._goal}")
                    self._goal_offset = None
                if ready:
                    payload_state = self._build_payload_state()
                    if self.controller is None:
                        formation = self._formation_from_poses(payload_state)
                        print(f"[central] formation calibrated from mocap: {formation}")
                        self.controller = MRCapController(
                            num_robots=self._n,
                            formation=formation,
                            config=self._controller_cfg,
                        )
                        self.controller.reset()
                    controls = self.controller.compute_control(
                        payload_state=payload_state,
                        robot_states=self._robot_states,
                        goal_state=self._goal,
                        dt=self._dt,
                        forces=self._forces,
                    )
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
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--horizon", type=int, default=15)
    parser.add_argument("--v-max", type=float, default=0.25)
    parser.add_argument("--gt-payload", action="store_true",
                        help="Use live mocap payload pose every step "
                             "(requires a 'payload' rigid body in mocap). "
                             "Default: estimator mode — payload pose synthesised "
                             "from robot positions at init only.")
    parser.add_argument("--relative-goal", action="store_true",
                        help="Treat --goal as an offset from the robots' initial "
                             "centroid position rather than an absolute world-frame target.")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    runner = CentralRunner(
        cfg, args.n_robots, np.array(args.goal),
        control_hz=args.control_hz,
        horizon=args.horizon,
        v_max=args.v_max,
        use_gt_payload=args.gt_payload,
        relative_goal=args.relative_goal,
    )
    runner.run()


if __name__ == "__main__":
    main()
