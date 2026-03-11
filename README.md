# Swarm Carry

Research platform for multi-robot cooperative payload transport. The stack
runs as pure Python (MuJoCo only) or over ROS2 Jazzy for sim-to-real transfer.

See [docs/scenario.md](docs/scenario.md) for the target research scenario and
current assumptions. The existing simulation infrastructure is a stepping stone —
read that doc before building on top of it.

---

## Repo structure

```
swarmlib/          Core Python library — no ROS2 required
  controllers/     BaseController interface + CentralizedMPC
  communication/   SimulatedBackend, AsyncSimulatedBackend, ROS2Backend
  simulation/      SwarmTransportEnv (MuJoCo step/reset), scene generator

src/               ROS2 workspace (colcon build runs here)
  swarm_mujoco_bridge/   MuJoCo physics exposed as ROS2 pub/sub
  swarm_agent/           Per-robot controller node
  swarm_central/         Centralized controller node

models/            Robot model assets
  turtlebot3/      TurtleBot3 Waffle Pi mesh files + standalone XML
  holonomic_dp/    Summit XL Steel model (git submodule, mecanum demo only)

experiments/
  mpc_scaling/     MPC solve-time vs n_robots scaling study
  gbp_estimation/  Gaussian Belief Propagation distributed estimation demos

demos/
  mujoco_swarm_demo/    Visual contact/physics validation (no ROS2)
  mecanum_ros2_demo/    Summit XL Steel mecanum drive via ROS2
```

---

## Setup

```bash
git clone <repo> && cd swarm-carry
git submodule update --init models/holonomic_dp

# Core library (MuJoCo + numpy — no ROS2 needed)
pip install -e swarmlib/

# ROS2 workspace (only needed for bridge/agent/central nodes)
source /opt/ros/jazzy/setup.bash
cd src && colcon build --symlink-install && source install/setup.bash
```

---

## Run a pure-Python simulation

```python
from swarmlib.simulation import SwarmTransportEnv
import numpy as np

env = SwarmTransportEnv(n_robots=4, goal_pos=[5.0, 0.0, 0.0])
obs = env.reset()

for _ in range(500):
    # obs keys: 'payload' (6,), 'robots' (n,4), 'forces' (n,3)
    controls = np.zeros((4, 2))   # [vx, vy] per robot, m/s world frame
    obs = env.step(controls)

env.close()
```

Scene XML is auto-generated and written to the system temp dir by default.
Pass `scenes_dir=Path("my/dir")` to save scenes for inspection.

---

## Write a controller

Subclass `BaseController`. The only requirement: `compute_control` returns
`(n, 2)` Cartesian `[vx, vy]` in m/s (world frame). Diff-drive conversion
is handled internally by the environment.

```python
from swarmlib.controllers import BaseController
import numpy as np

class MyController(BaseController):
    def compute_control(self, payload_state, robot_states, goal_state,
                        dt, forces=None):
        # payload_state: (6,) [x, y, theta, vx, vy, omega]
        # robot_states:  (n, 4) [x, y, vx, vy]
        # goal_state:    (3,) [x_goal, y_goal, theta_goal]
        # forces:        (n, 3) [fx, fy, torque_z]
        return np.zeros((self.num_robots, 2))

    def reset(self):
        pass
```

Plug it into a node by setting `self.controller = MyController(...)` in
`src/swarm_agent/swarm_agent/agent_controller_node.py` or
`src/swarm_central/swarm_central/central_controller_node.py`.

---

## Communication backend (distributed algorithms)

`CommunicationBackend` handles peer-to-peer algorithm messages (GBP beliefs,
factor graph messages, etc.) independently of the physics bridge.

```python
from swarmlib.communication.backend import SimulatedBackend, create_ring_topology
from swarmlib.communication.ros2_backend import ROS2Backend

topo = create_ring_topology(4)

# Testing — no ROS2 needed:
backend = SimulatedBackend(num_agents=4, topology=topo)

# Real networked deployment:
backend = ROS2Backend(num_agents=4, topology=topo)

# API is identical:
backend.broadcast(from_id=0, message=my_gaussian_msg)
backend.barrier()
messages = backend.receive(agent_id=1)
```

See `experiments/gbp_estimation/` for working examples with dropout and delay.

---

## Full ROS2 simulation

**Terminal 1 — physics bridge:**
```bash
source /opt/ros/jazzy/setup.bash && source src/install/setup.bash
ros2 launch swarm_mujoco_bridge sim.launch.py n_robots:=2 push_distance:=5.0
```

Verify it's running:
```bash
ros2 topic echo /swarm/payload/state
ros2 topic echo /swarm/robot_0/force
```

**Terminal 2 — controller (pick one):**
```bash
# Centralized (laptop runs one node for all robots):
ros2 launch swarm_central central.launch.py n_robots:=2 goal_x:=5.0

# Decentralized (one node per robot, same machine or distributed):
ros2 launch swarm_agent agent.launch.py agent_id:=0 neighbor_ids:="1" goal_x:=5.0
ros2 launch swarm_agent agent.launch.py agent_id:=1 neighbor_ids:="0" goal_x:=5.0
```

Controller nodes do not know whether they talk to sim or real hardware —
the topic interface is identical.

---

## Topic map

```
/swarm/payload/state        ← bridge (sim) or payload sensor node (real)
/swarm/robot_{i}/state      ← bridge (sim) or robot odometry (real)
/swarm/robot_{i}/force      ← bridge (sim) or F/T sensor (real)
/swarm/robot_{i}/cmd_vel    → applied to MuJoCo actuators (sim) or motor controller (real)
/swarm/agent_{i}/outbox     ← GBP / distributed algorithm peer messages
```

---

## Real robot deployment (TurtleBot3)

| Machine | Process |
|---------|---------|
| Laptop | `swarm_mujoco_bridge` (sim) or nothing (real hardware) |
| Each TurtleBot3 | `swarm_agent agent_node` — one per robot |
| Laptop or any machine | `swarm_central central_node` (centralized methods only) |

```bash
# On each TurtleBot3:
ros2 launch swarm_agent agent.launch.py agent_id:=0 neighbor_ids:="1,2" goal_x:=5.0

# On laptop (centralized):
ros2 launch swarm_central central.launch.py n_robots:=3 goal_x:=5.0
```

---

## Experiments

```bash
# MPC scaling study (solve time vs number of robots):
cd experiments/mpc_scaling
python run_scaling_experiment.py --n-values 2,4,8 --distance 5.0

# GBP distributed estimation demos:
cd experiments/gbp_estimation
python gbp_distributed_estimation.py
python test_async_dropout.py
```

---

## Demos

```bash
# Physics/contact validation (no ROS2):
cd demos/mujoco_swarm_demo
python scripts/demo.py

# Mecanum drive (requires ROS2 + holonomic_dp submodule):
cd demos/mecanum_ros2_demo
python demo.py
```

---

## Notes for teammates

- After pulling: `git submodule sync && git submodule update --init models/holonomic_dp`
- The `swarmlib` venv install is editable (`pip install -e`), so local edits
  take effect immediately without reinstalling.
- ROS2 packages import from `swarmlib` directly — no `sys.path` hacks needed.
- `colcon build` only covers `src/`. Everything else (`swarmlib`, `experiments`,
  `demos`) is plain Python.
