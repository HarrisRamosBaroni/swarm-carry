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
import signal
import time

import yaml
import zmq
import numpy as np

from real_robot.transport.messages import force_msg, unpack
from real_robot.robot.ros1_bridge import ROS1Bridge
from real_robot.robot.load_cell_reader import LoadCellReader
from swarmlib.communication.zmq_backend import ZeroMQSingleAgentBackend


PAYLOAD_ID = -1  # sentinel id mocap_bridge uses for the payload rigid body


class AgentRunner:
    def __init__(self, robot_id: int, neighbor_ids: list,
                 network_config: dict, goal: np.ndarray,
                 n_robots: int,
                 control_hz: float = 20.0,
                 gbp_async: bool = False,
                 gbp_max_iters: int = 30,
                 horizon: int = 15,
                 v_max: float = 0.25,
                 passive: bool = False):
        self._id = robot_id
        self._neighbors = neighbor_ids
        self._goal = goal
        self._dt = 1.0 / control_hz
        self._n_robots = n_robots
        self._passive = passive

        cfg = network_config
        ctx = zmq.Context.instance()

        # PUB: broadcast our state + force to laptop and peers
        my_port = next(r["pub_port"] for r in cfg["robots"] if r["id"] == robot_id)
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{my_port}")

        # SUB: mocap poses + commands + goal updates from laptop
        self._sub = ctx.socket(zmq.SUB)
        self._sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
        self._sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['central_pub_port']}")
        self._sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['goal_pub_port']}")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "pose")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "cmd")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "goal")

        # Also subscribe to neighbor force for decentralised controller
        if not passive:
            for nid in neighbor_ids:
                nip = next(r["ip"] for r in cfg["robots"] if r["id"] == nid)
                nport = next(r["pub_port"] for r in cfg["robots"] if r["id"] == nid)
                self._sub.connect(f"tcp://{nip}:{nport}")
                self._sub.setsockopt_string(zmq.SUBSCRIBE, "force")

        # GBP backend — only needed for the decentralised controller. In
        # passive mode the laptop is the brain; we just publish state and
        # forward cmd → cmd_vel.
        self.backend = None
        if not passive:
            self.backend = ZeroMQSingleAgentBackend(
                my_id=robot_id,
                neighbors=neighbor_ids,
                network_config=cfg,
                synchronous=not gbp_async,
            )

        # Local ROS1 bridge (cmd_vel only — poses come from mocap)
        self._ros = ROS1Bridge(node_name=f"swarm_agent_{robot_id}")

        # Load cells
        self._lc = LoadCellReader()
        self._lc.tare()
        print(f"[agent {robot_id}] load cell tared")

        # State buffers — all poses come from mocap bridge
        self._poses = {}           # robot_id → latest pose message dict
        self._own_prev_pose = None  # for velocity differentiation
        self._payload_state = np.zeros(6)

        # Diagnostic state
        self._got_own_pose = False
        self._got_payload_pose = False
        self._printed_waiting = False
        self._cmd_count = 0
        self._last_heartbeat = 0.0

        self.controller = None
        self._controller_cfg = {
            "horizon": horizon,
            "v_max": v_max,
            "sigma_x": 0.5,
            "sigma_u": 0.3,
            "sigma_anchor": 0.01,
            "sigma_r2r": 0.05,
            "sigma_pull_in": 0.3,
            "sigma_consensus": 0.1,
            "gbp_max_iters": gbp_max_iters,
            "gbp_tol": 1e-3,
        }

        self._running = True
        signal.signal(signal.SIGINT, self._handle_sigint)
        signal.signal(signal.SIGTERM, self._handle_sigint)

        laptop_ip = cfg['laptop']['ip']
        mode = "passive" if passive else "active"
        print(f"[agent {robot_id}] ready — mode={mode}, pub_port={my_port}, "
              f"laptop={laptop_ip}, neighbors={neighbor_ids}")

        time.sleep(0.2)

    def _handle_sigint(self, sig, frame):
        print(f"\n[agent {self._id}] shutting down")
        self._running = False

    def _formation_from_poses(self) -> list:
        pp = self._poses[PAYLOAD_ID]
        p_c = np.array([pp["x"], pp["y"]])
        theta = float(pp["theta"])
        c, s = np.cos(theta), np.sin(theta)
        R_inv = np.array([[c, s], [-s, c]])
        offsets = []
        for i in range(self._n_robots):
            rp = self._poses[i]
            r_body = R_inv @ (np.array([rp["x"], rp["y"]]) - p_c)
            offsets.append((float(r_body[0]), float(r_body[1]), 0.0))
        return offsets

    def run(self):
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)
        next_tick = time.monotonic()

        while self._running:
            # Drain incoming messages
            while self._running and dict(poller.poll(timeout=0)):
                topic_bytes, raw = self._sub.recv_multipart()
                d = unpack(raw)
                t = d.get("t")
                if t == "goal":
                    self._goal = np.array([d["x"], d["y"], d["theta"]])
                    print(f"[agent {self._id}] goal updated to "
                          f"({d['x']:.2f}, {d['y']:.2f}, {d['theta']:.2f} rad) "
                          f"tol={d['tol']:.2f} m")
                elif t == "pose":
                    rid = d["id"]
                    self._poses[rid] = d
                    if rid == self._id and not self._got_own_pose:
                        self._got_own_pose = True
                        print(f"[agent {self._id}] first own pose received "
                              f"x={d['x']:.2f} y={d['y']:.2f}")
                    if rid == PAYLOAD_ID and not self._got_payload_pose:
                        self._got_payload_pose = True
                        print(f"[agent {self._id}] first payload pose received "
                              f"x={d['x']:.2f} y={d['y']:.2f}")
                elif t == "cmd" and d.get("id") == self._id:
                    self._ros.send_cmd(d["vx"], d["vy"])
                    self._cmd_count += 1

            self._ros.spin_once()

            now = time.monotonic()
            if now >= next_tick:
                lc_readings = self._lc.read()
                raw_force = force_msg(self._id, lc_readings)
                self._pub.send_multipart([b"force", raw_force])

                if not self._passive:
                    # All poses come from mocap bridge. Skip tick until own
                    # pose, payload pose, and all peer poses have arrived.
                    own = self._poses.get(self._id)
                    pp = self._poses.get(PAYLOAD_ID)
                    all_peers = all(i in self._poses for i in range(self._n_robots))
                    if own is None or pp is None or not all_peers:
                        if not self._printed_waiting:
                            missing = ([f"own(id={self._id})"] if own is None else [])
                            missing += (["payload"] if pp is None else [])
                            missing += [f"peer {i}" for i in range(self._n_robots)
                                        if i not in self._poses]
                            print(f"[agent {self._id}] waiting for: {', '.join(missing)}")
                            self._printed_waiting = True
                        next_tick += self._dt
                        time.sleep(max(0.0, next_tick - time.monotonic()))
                        continue
                    if self._printed_waiting:
                        print(f"[agent {self._id}] all poses received, starting control")
                        self._printed_waiting = False

                    if self.controller is None:
                        from swarmlib.controllers import DRCapDistributedController
                        formation = self._formation_from_poses()
                        print(f"[agent {self._id}] formation calibrated from mocap: {formation}")
                        self.controller = DRCapDistributedController(
                            num_robots=self._n_robots,
                            formation=formation,
                            backend=self.backend,
                            my_id=self._id,
                            config=self._controller_cfg,
                        )
                        self.controller.reset()

                    # Differentiate own mocap pose for velocity
                    if self._own_prev_pose is not None:
                        dt = own["ts"] - self._own_prev_pose["ts"]
                        if dt > 0:
                            vx = (own["x"] - self._own_prev_pose["x"]) / dt
                            vy = (own["y"] - self._own_prev_pose["y"]) / dt
                        else:
                            vx = vy = 0.0
                    else:
                        vx = vy = 0.0
                    self._own_prev_pose = own

                    self._payload_state[0] = pp["x"]
                    self._payload_state[1] = pp["y"]
                    self._payload_state[2] = pp["theta"]

                    robot_states = np.array([[own["x"], own["y"], vx, vy]])
                    controls = self.controller.compute_control(
                        payload_state=self._payload_state,
                        robot_states=robot_states,
                        goal_state=self._goal,
                        dt=self._dt,
                        forces=None,  # expand once load cell format is finalised
                    )
                    self._ros.send_cmd(float(controls[0, 0]), float(controls[0, 1]))

                now2 = time.monotonic()
                if now2 - self._last_heartbeat >= 5.0:
                    self._last_heartbeat = now2
                    if self._passive:
                        print(f"[agent {self._id}] heartbeat — cmds forwarded: {self._cmd_count}")
                        self._cmd_count = 0
                    else:
                        own = self._poses.get(self._id, {})
                        print(f"[agent {self._id}] heartbeat — "
                              f"pos=({own.get('x', float('nan')):.2f}, {own.get('y', float('nan')):.2f}) "
                              f"cmd=({controls[0, 0]:.3f}, {controls[0, 1]:.3f})")

                next_tick += self._dt

            time.sleep(max(0.0, next_tick - time.monotonic()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/home/ubuntu/network.yaml")
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--neighbors", type=int, nargs="*", default=[])
    parser.add_argument("--goal", type=float, nargs=3, default=[5.0, 0.0, 0.0])
    parser.add_argument("--n-robots", type=int, default=1,
                        help="Total robots in formation. Required for decentralised "
                             "mode; ignored in --passive mode.")
    parser.add_argument("--passive", action="store_true",
                        help="State publisher + cmd forwarder only — no local "
                             "controller. Use this when the laptop runs the "
                             "centralised controller.")
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--horizon", type=int, default=15)
    parser.add_argument("--v-max", type=float, default=0.25)
    parser.add_argument("--gbp-max-iters", type=int, default=30)
    parser.add_argument("--gbp-async", action="store_true",
                        help="Use asynchronous GBP (non-blocking barrier, uses stale beliefs)")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    runner = AgentRunner(
        robot_id=args.id,
        neighbor_ids=args.neighbors,
        network_config=cfg,
        goal=np.array(args.goal),
        n_robots=args.n_robots,
        control_hz=args.control_hz,
        gbp_async=args.gbp_async,
        gbp_max_iters=args.gbp_max_iters,
        horizon=args.horizon,
        v_max=args.v_max,
        passive=args.passive,
    )
    runner.run()


if __name__ == "__main__":
    main()
