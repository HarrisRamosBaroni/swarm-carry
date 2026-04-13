# Real-Robot Deployment — Implementation Plan (Option C)

## Architecture decision

**Option C: ROS1 local-only on each robot + ZeroMQ across machines.**

- Each myAGV runs Ubuntu 18.04 / ROS1 Melodic. The `myagv_ros` stack runs entirely
  locally on each robot for wheel odometry and base velocity control. It never
  leaves the robot.
- All cross-machine communication (robot↔laptop, robot↔robot) uses ZeroMQ.
- The laptop (Ubuntu 24.04) runs the PhaseSpace mocap ROS2 node (`swarm_mocap`)
  and a thin bridge that rebroadcasts poses over ZeroMQ.
- The MuJoCo simulation stack (`swarmlib/`, `src/`) is untouched — it stays as-is
  for development and testing.

### Why not ROS1 port / ros1_bridge

- ROS1 Melodic is EOL (May 2023). Porting `src/` to rospy + catkin is non-trivial
  work on a tight deadline.
- `ros1_bridge` is fragile and routes all peer messages through `rosmaster` — a
  single point of failure that undermines the decentralised design.
- ROS1 Melodic cannot be installed natively on the Ubuntu 24.04 laptop.

---

## What already exists (do not change)

- `swarmlib/` — all controller logic, simulation environments, `CommunicationBackend`
  interface and existing backends. Import freely; do not modify.
- `src/` — ROS2 nodes for simulation-only use. Leave as-is.
- `src/swarm_mocap/` — PhaseSpace ROS2 driver. Runs on the laptop unchanged.

---

## Repository layout for new code

```
real_robot/
  config/
    network.yaml              # IP addresses, ports, robot IDs, mocap rigid body IDs
  transport/
    messages.py               # Msgpack message schemas + serialization helpers
  swarmlib/communication/
    zmq_backend.py            # ZeroMQSingleAgentBackend (new CommunicationBackend)
  laptop/
    mocap_bridge.py           # ROS2 /mocap topics → ZeroMQ PUB
    central_runner.py         # Centralized controller over ZeroMQ (no ROS)
  robot/
    agent_runner.py           # Decentralised agent runner (runs on each myAGV)
    ros1_bridge.py            # rospy interface to local myagv_ros odom + cmd_vel
    load_cell_reader.py       # HX711 GPIO reader (stub — fill in with hardware team)
```

Note: `zmq_backend.py` lives in `swarmlib/communication/` so it is importable
everywhere (laptop and robot) without path hacks.

---

## Network configuration — `real_robot/config/network.yaml`

```yaml
robots:
  - id: 0
    ip: "192.168.0.101"   # fill in actual IPs
    pub_port: 5550
  - id: 1
    ip: "192.168.0.102"
    pub_port: 5551
  - id: 2
    ip: "192.168.0.103"
    pub_port: 5552
  - id: 3
    ip: "192.168.0.104"
    pub_port: 5553

laptop:
  ip: "192.168.0.200"     # fill in actual IP
  mocap_pub_port: 5560    # PUB: laptop broadcasts all rigid body poses
  central_pub_port: 5561  # PUB: centralized controller broadcasts cmd_vel

control:
  frequency_hz: 20        # start conservative; increase after validation

mocap:
  rigid_body_ids:
    robot_0: 1            # PhaseSpace rigid body ID → robot ID mapping
    robot_1: 2
    robot_2: 3
    robot_3: 4
    payload: 5
```

---

## ZeroMQ topology

Every node (laptop + each robot) binds **one PUB socket** on its own port and
connects **SUB sockets** to all nodes it needs to receive from.

Topic prefixes (ZeroMQ subscription filter strings):
- `pose`   — rigid body pose from mocap bridge (laptop → everyone)
- `state`  — robot odometry state (robot → laptop + peers)
- `force`  — load cell reading (robot → laptop + peers)
- `cmd`    — velocity command (laptop → robot for centralised case)
- `peer`   — GBP / distributed algorithm message (robot → specific peer)

All messages are **msgpack-serialised dicts**. See `messages.py` below.

---

## Step 1 — `real_robot/transport/messages.py`

Serialisation helpers used by every component. No ZeroMQ dependency here.

```python
"""
Message schemas for real-robot ZeroMQ transport.
All messages are msgpack-serialised dicts. Import this on both laptop and robot.
"""
import time
import msgpack
import numpy as np

# --- Schemas (as plain dicts; no dataclasses to keep rospy-compatible) ---

def pose_msg(robot_id: int, x: float, y: float, theta: float) -> bytes:
    return msgpack.packb({
        "t": "pose",
        "id": robot_id,
        "ts": time.time(),
        "x": x, "y": y, "theta": theta,
    })

def state_msg(robot_id: int, x: float, y: float,
              vx: float, vy: float, theta: float, omega: float) -> bytes:
    return msgpack.packb({
        "t": "state",
        "id": robot_id,
        "ts": time.time(),
        "x": x, "y": y, "vx": vx, "vy": vy, "theta": theta, "omega": omega,
    })

def force_msg(robot_id: int, readings: list) -> bytes:
    """
    readings: list of {"label": str, "value": float} dicts, one per load cell.
    Format TBD pending physical mounting geometry — hardware team fills this in.
    Example: [{"label": "lc_base", "value": 12.3}, {"label": "lc_wall_x", "value": -0.4}]
    """
    return msgpack.packb({
        "t": "force",
        "id": robot_id,
        "ts": time.time(),
        "readings": readings,
    })

def cmd_msg(robot_id: int, vx: float, vy: float) -> bytes:
    return msgpack.packb({
        "t": "cmd",
        "id": robot_id,
        "vx": vx, "vy": vy,
    })

def peer_msg(from_id: int, to_id: int, epoch: int, payload: bytes) -> bytes:
    """payload is already serialised (e.g. msgpack bytes of GaussianMessage fields)."""
    return msgpack.packb({
        "t": "peer",
        "from": from_id,
        "to": to_id,
        "epoch": epoch,
        "payload": payload,
    })

def unpack(raw: bytes) -> dict:
    return msgpack.unpackb(raw, raw=False)
```

---

## Step 2 — `swarmlib/communication/zmq_backend.py`

Drop-in replacement for `SingleAgentROS2Backend`. Implements `CommunicationBackend`
using ZeroMQ PUB/SUB so the same GBP / distributed algorithm code runs over the
network without ROS2.

```python
"""
ZeroMQ single-agent communication backend.

Drop-in replacement for SingleAgentROS2Backend for real-robot deployment.
Implements CommunicationBackend using ZeroMQ PUB/SUB. No ROS dependency.

Usage (on each robot):
    from swarmlib.communication.zmq_backend import ZeroMQSingleAgentBackend
    backend = ZeroMQSingleAgentBackend(
        my_id=0,
        neighbors=[1, 2],
        network_config=config,   # loaded from real_robot/config/network.yaml
    )
    # API is identical to SingleAgentROS2Backend / SimulatedBackend
"""
import time
import msgpack
import numpy as np
import zmq
from typing import List, Tuple, Dict, Any

from swarmlib.communication.backend import CommunicationBackend, GaussianMessage


class ZeroMQSingleAgentBackend(CommunicationBackend):
    """
    One-agent-per-process ZeroMQ communication backend.

    Each robot process binds a PUB socket on its own port and connects
    SUB sockets to each neighbor's PUB port. GaussianMessage peer messages
    are serialised with msgpack.

    Parameters
    ----------
    my_id : int
    neighbors : list of int
    network_config : dict
        Parsed real_robot/config/network.yaml. Used to look up IPs and ports.
    barrier_timeout : float
        Seconds to wait in barrier() before raising TimeoutError.
    """

    def __init__(
        self,
        my_id: int,
        neighbors: List[int],
        network_config: dict,
        barrier_timeout: float = 5.0,
    ):
        topology = {my_id: list(neighbors)}
        super().__init__(num_agents=1, topology=topology)

        self.my_id = my_id
        self._neighbors = list(neighbors)
        self._barrier_timeout = barrier_timeout
        self._current_epoch = 0
        self._inbox: List[Tuple[int, GaussianMessage]] = []
        self._received_count = 0
        self._expected = len(neighbors)

        ctx = zmq.Context.instance()

        # Bind PUB socket on our own port
        my_port = _robot_port(network_config, my_id)
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{my_port}")

        # Connect SUB sockets to each neighbor
        self._subs: List[zmq.Socket] = []
        for nid in neighbors:
            sub = ctx.socket(zmq.SUB)
            nip = _robot_ip(network_config, nid)
            nport = _robot_port(network_config, nid)
            sub.connect(f"tcp://{nip}:{nport}")
            sub.setsockopt_string(zmq.SUBSCRIBE, f"peer:{my_id}:")
            self._subs.append(sub)

        # Poller for non-blocking receive
        self._poller = zmq.Poller()
        for sub in self._subs:
            self._poller.register(sub, zmq.POLLIN)

        time.sleep(0.1)  # allow ZeroMQ connections to establish

    # --- Serialisation -------------------------------------------------------

    def _pack(self, to_id: int, message: GaussianMessage) -> bytes:
        return msgpack.packb({
            "from": self.my_id,
            "to": to_id,
            "epoch": message.epoch,
            "eta": message.eta.tolist(),
            "lam": message.lam.tolist(),
        })

    @staticmethod
    def _unpack(raw: bytes) -> Tuple[int, int, GaussianMessage]:
        d = msgpack.unpackb(raw, raw=False)
        return (
            d["from"],
            d["to"],
            GaussianMessage(
                eta=np.array(d["eta"]),
                lam=np.array(d["lam"]),
                epoch=d["epoch"],
            ),
        )

    # --- CommunicationBackend interface --------------------------------------

    def send(self, from_id: int, to_id: int, message: Any) -> None:
        if from_id != self.my_id:
            raise ValueError(f"from_id must be {self.my_id}")
        stamped = GaussianMessage(
            eta=message.eta.copy(), lam=message.lam.copy(),
            epoch=self._current_epoch,
        )
        topic = f"peer:{to_id}:{self.my_id}".encode()
        self._pub.send_multipart([topic, self._pack(to_id, stamped)])
        self._stats["messages_sent"] += 1

    def receive(self, agent_id: int) -> List[Tuple[int, GaussianMessage]]:
        # Drain any waiting messages before returning
        while True:
            ready = dict(self._poller.poll(timeout=0))
            if not ready:
                break
            for sub in self._subs:
                if sub in ready:
                    _, raw = sub.recv_multipart()
                    from_id, to_id, gmsg = self._unpack(raw)
                    if to_id == self.my_id:
                        self._inbox.append((from_id, gmsg))
                        self._received_count += 1

        messages = list(self._inbox)
        self._inbox.clear()
        self._received_count = 0
        return messages

    def broadcast(self, from_id: int, message: Any) -> None:
        for nid in self._neighbors:
            self.send(from_id, nid, message)

    def barrier(self) -> None:
        deadline = time.monotonic() + self._barrier_timeout
        while self._received_count < self._expected:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"ZeroMQSingleAgentBackend barrier timeout. "
                    f"Received {self._received_count}/{self._expected}."
                )
            ready = dict(self._poller.poll(timeout=5))
            for sub in self._subs:
                if sub in ready:
                    _, raw = sub.recv_multipart()
                    from_id, to_id, gmsg = self._unpack(raw)
                    if to_id == self.my_id:
                        self._inbox.append((from_id, gmsg))
                        self._received_count += 1
        self._current_epoch += 1
        self._stats["barrier_calls"] += 1

    @property
    def is_synchronous(self) -> bool:
        return True

    def shutdown(self) -> None:
        self._pub.close()
        for sub in self._subs:
            sub.close()


# --- Helpers -----------------------------------------------------------------

def _robot_ip(cfg: dict, robot_id: int) -> str:
    for r in cfg["robots"]:
        if r["id"] == robot_id:
            return r["ip"]
    raise KeyError(f"Robot {robot_id} not in network config")

def _robot_port(cfg: dict, robot_id: int) -> int:
    for r in cfg["robots"]:
        if r["id"] == robot_id:
            return r["pub_port"]
    raise KeyError(f"Robot {robot_id} not in network config")
```

---

## Step 3 — `real_robot/robot/ros1_bridge.py`

Runs on each myAGV. Opens a rospy subscriber for odometry and a publisher for
cmd_vel. The agent runner calls into this; it never touches ZeroMQ directly.

```python
"""
ROS1 (rospy) bridge — runs on each myAGV alongside myagv_ros.

Provides:
  - get_odom()  → dict {x, y, vx, vy, theta, omega}
  - send_cmd(vx, vy)  → publishes to /cmd_vel

Call spin_once() in the agent's control loop to process incoming callbacks.
"""
import threading
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class ROS1Bridge:
    def __init__(self, node_name: str = "swarm_agent_ros1"):
        rospy.init_node(node_name, anonymous=False)
        self._odom = {"x": 0.0, "y": 0.0, "vx": 0.0, "vy": 0.0,
                      "theta": 0.0, "omega": 0.0}
        self._lock = threading.Lock()

        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=1)
        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)
        with self._lock:
            self._odom = {
                "x":     msg.pose.pose.position.x,
                "y":     msg.pose.pose.position.y,
                "vx":    msg.twist.twist.linear.x,
                "vy":    msg.twist.twist.linear.y,
                "theta": theta,
                "omega": msg.twist.twist.angular.z,
            }

    def get_odom(self) -> dict:
        with self._lock:
            return dict(self._odom)

    def send_cmd(self, vx: float, vy: float) -> None:
        """Send mecanum velocity command in robot frame."""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        self._cmd_pub.publish(twist)

    def spin_once(self) -> None:
        rospy.rostime.wallsleep(0)  # process pending callbacks without blocking
```

---

## Step 4 — `real_robot/robot/load_cell_reader.py`

**Stub — to be completed by hardware team after load cells are mounted.**

The hardware team should fill in `read()` to return one dict per load cell.
Label names and which axis each cell measures are determined by physical mounting.

```python
"""
Load cell reader — HX711 via GPIO on Raspberry Pi.

HARDWARE TEAM: fill in read() once load cells are wired up.
Each entry in the returned list is {"label": <str>, "value": <float, Newtons>}.
Label naming convention to discuss with software team so labels match
what the controller expects.

Example expected output for two load cells:
  [{"label": "base", "value": 42.1}, {"label": "wall_x", "value": -1.3}]
"""


class LoadCellReader:
    def __init__(self):
        # HARDWARE TEAM: initialise HX711 channels here
        # e.g.: from hx711 import HX711
        #       self._hx = HX711(dout_pin=..., pd_sck_pin=...)
        #       self._hx.set_reading_format(...)
        #       self._tare = self._hx.get_raw_data_mean()
        self._tare = 0.0  # placeholder

    def read(self) -> list:
        """
        Return list of {"label": str, "value": float} dicts.
        Blocking read — call from control loop at desired rate.
        """
        # HARDWARE TEAM: replace with real HX711 read + tare subtraction
        raise NotImplementedError("LoadCellReader.read() not yet implemented")

    def tare(self) -> None:
        """Zero out current reading. Call once at startup."""
        raise NotImplementedError
```

---

## Step 5 — `real_robot/laptop/mocap_bridge.py`

Runs on the laptop. Subscribes to the ROS2 `/mocap/rigid_{id}` topics published
by `swarm_mocap` and rebroadcasts all poses over a ZeroMQ PUB socket.

```python
"""
Mocap bridge — laptop only.

Subscribes to /mocap/rigid_{id} ROS2 topics (from swarm_mocap node).
Rebroadcasts all rigid body poses over ZeroMQ so robots can subscribe.

Run AFTER swarm_mocap is already publishing:
  ros2 launch swarm_mocap mocap.launch.py server_ip:=192.168.0.244

Then:
  python real_robot/laptop/mocap_bridge.py --config real_robot/config/network.yaml
"""
import argparse
import yaml
import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

from real_robot.transport.messages import pose_msg


class MocapBridge(Node):
    def __init__(self, network_config: dict, rigid_body_ids: dict):
        super().__init__("mocap_zmq_bridge")

        ctx = zmq.Context.instance()
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{network_config['laptop']['mocap_pub_port']}")

        # Subscribe to each rigid body's per-body topic
        for robot_id, rb_id in rigid_body_ids.items():
            if not robot_id.startswith("robot_"):
                continue
            rid = int(robot_id.split("_")[1])
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
```

---

## Step 6 — `real_robot/laptop/central_runner.py`

Centralised controller. No ROS. Subscribes to state + force from all robots over
ZeroMQ. Runs `swarmlib` controller. Publishes cmd_vel to each robot.

```python
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
import msgpack

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
```

---

## Step 7 — `real_robot/robot/agent_runner.py`

Runs on each myAGV. Uses `ros1_bridge.py` for local odom + cmd_vel, ZeroMQ for
everything cross-machine.

```python
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
```

---

## Testing without hardware

These components can be exercised on the laptop alone before touching any myAGV:

| Component | How to test |
|---|---|
| `zmq_backend.py` | Run two Python processes on localhost: one as robot 0, one as robot 1. Send GBP messages between them. |
| `messages.py` | Unit test: pack/unpack round-trip for each message type. |
| `mocap_bridge.py` | Mock the `/mocap/rigid_{id}` topics with a dummy ROS2 publisher; check ZeroMQ subscriber receives correct pose dicts. |
| `central_runner.py` | Mock robot state publishers from two terminal processes; check controller tick fires at expected rate. |
| `agent_runner.py` (partial) | Run without `ros1_bridge` and `load_cell_reader` (mock those); test ZeroMQ pub/sub and controller integration. |

---

## Hardware team interface

Tell your teammates the following:

> The load cell reader must provide a `read()` method returning a list of dicts:
> `[{"label": "<name>", "value": <float, Newtons>}, ...]`
> — one entry per physical load cell.
> Label names are up to you based on mounting geometry; just document which label
> corresponds to which axis/position. We will adapt the controller to consume
> these labels once the physical design is fixed.
> Connect each HX711 to the Raspberry Pi GPIO and implement in
> `real_robot/robot/load_cell_reader.py`.

---

## Run order for a real experiment

**Terminal on laptop (once):**
```bash
# 1. Start mocap
source /opt/ros/jazzy/setup.bash && source src/install/setup.bash
ros2 launch swarm_mocap mocap.launch.py server_ip:=192.168.0.244

# 2. Start mocap ZeroMQ bridge (separate terminal)
python real_robot/laptop/mocap_bridge.py --config real_robot/config/network.yaml

# 3a. Centralised — start controller
python real_robot/laptop/central_runner.py --config real_robot/config/network.yaml --n-robots 2 --goal 5.0 0.0 0.0
```

**On each myAGV (SSH):**
```bash
# 4. Ensure myagv_ros is running (provides /odom and accepts /cmd_vel)
roslaunch myagv_ros myagv_active.launch

# 5. Start agent
python real_robot/robot/agent_runner.py --config /home/ubuntu/network.yaml --id 0 --neighbors 1 --goal 5.0 0.0 0.0
```

For decentralised, skip step 3a — each robot runs its own controller in step 5.

---

## Dependencies to add to requirements.txt

```
pyzmq>=25.0
msgpack>=1.0
pyyaml>=6.0
```
