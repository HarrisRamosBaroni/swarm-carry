# Swarm Carry

Research platform for multi-robot cooperative payload transport. The stack
runs as pure Python (MuJoCo only) or on real myAGV robots via ZeroMQ (no ROS2
required anywhere).

See [docs/scenario.md](docs/scenario.md) for the target research scenario and
current assumptions. The existing simulation infrastructure is a stepping stone —
read that doc before building on top of it.

---

## Repo structure

```
swarmlib/          Core Python library
  controllers/     BaseController interface + controllers (MPC, MRCap, DRCap, …)
  communication/   SimulatedBackend, AsyncSimulatedBackend, ZeroMQSingleAgentBackend
  simulation/      MuJoCo environments and scene generators
    env.py             SwarmTransportEnv — diff-drive TurtleBot3s all on one face,
                       straight-line push; used only for the MPC scaling experiment
    holonomic_env.py   HolonomicTransportEnv — abstract kinematic placeholder
                       (box robots, no wheel physics); not used for the actual scenario
    mecanum_env.py     MecanumTransportEnv — primary env for the actual research
                       scenario: mecanum-wheeled robots with L-carriages, configurable
                       formation around the payload centroid, base/wall force readout

real_robot/        Real-robot deployment stack (ZeroMQ + ROS1 locally on myAGVs)
  laptop/          Laptop-side runners: central_runner, mocap_bridge, control_panel,
                   record, analyse_recordings
  robot/           Per-robot process: agent_runner, ros1_bridge, load_cell_reader
  transport/       ZMQ message definitions
  scripts/         deploy.sh, launch.sh, mocap_pub.py (OWL → ZMQ)
  config/          network.yaml.example, dev.env.example

models/            Robot model assets
  turtlebot3/      TurtleBot3 Waffle Pi mesh files + standalone XML
  holonomic_dp/    Summit XL Steel model (git submodule, mecanum demo only)

experiments/
  mpc_scaling/     MPC solve-time vs n_robots scaling study
  gbp_estimation/  Gaussian Belief Propagation distributed estimation demos
  drcap_fg/        DR.CAP distributed controller experiments
  mrcap_fg/        MR.CAP centralised controller experiments
  …

demos/
  mujoco_swarm_demo/    Visual contact/physics validation
  research_scenario_demo/  Full mecanum research scenario demo
```

---

## Setup

```bash
git clone <repo> && cd swarm-carry
git submodule update --init models/holonomic_dp

# All Python dependencies + editable swarmlib install
pip install -r requirements.txt
```

For real-robot deployment, additional per-machine setup is documented in
[real_robot/README.md](real_robot/README.md).

---

## Run a pure-Python simulation

`SwarmTransportEnv` is the preliminary side-push environment used only in the
MPC scaling experiment — all robots are on one face of the payload, so it is
incompatible with any centroid-based formation controller (mr.cap, factor graph,
GBP). `MecanumTransportEnv` is the environment for the actual research scenario:
mecanum-wheeled robots with L-shaped forklift carriages arranged around the payload
centroid, with `base_forces` (vertical load) and `wall_forces` (shear) readout.

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

For the actual scenario, use `MecanumTransportEnv`:

```python
from swarmlib.simulation import MecanumTransportEnv
import numpy as np

env = MecanumTransportEnv(n_robots=4, goal=(5.0, 0.0, 0.0))
obs = env.reset()

for _ in range(500):
    # obs keys: 'payload' (6,), 'robots' (n,4),
    #           'base_forces' (n,) Fz scalar/robot, 'wall_forces' (n,) Fx scalar/robot
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

For simulation experiments, instantiate the controller directly in your
`experiments/<method>/run_experiment.py`. For real-robot deployment, wire
it into `real_robot/robot/agent_runner.py` (decentralised) or
`real_robot/laptop/central_runner.py` (centralised) — see
[real_robot/README.md](real_robot/README.md) for the sim→deploy pattern.

---

## Communication backend (distributed algorithms)

`CommunicationBackend` handles peer-to-peer algorithm messages (GBP beliefs,
factor graph messages, etc.) independently of the physics bridge.

```python
from swarmlib.communication.backend import SimulatedBackend, create_ring_topology
from swarmlib.communication.zmq_backend import ZeroMQSingleAgentBackend

topo = create_ring_topology(4)

# Testing — pure Python, no networking:
backend = SimulatedBackend(num_agents=4, topology=topo)

# Real deployment (one process per robot):
backend = ZeroMQSingleAgentBackend(my_id=0, topology=topo)

# API is identical:
backend.broadcast(from_id=0, message=my_gaussian_msg)
backend.barrier()
messages = backend.receive(agent_id=1)
```

See `experiments/gbp_estimation/` for working examples with dropout and delay.

---

## Real robot deployment (myAGV)

The real-robot stack lives entirely in `real_robot/` and uses ZeroMQ for
cross-machine communication. ROS1 Melodic runs locally on each myAGV only
for `/cmd_vel` delivery; the laptop needs no ROS at all.

| Machine | Process |
|---------|---------|
| Laptop | `mocap_pub.py` (PhaseSpace → ZMQ), `central_runner.py` or `control_panel.py` |
| Each myAGV | `agent_runner.py` + `ros1_bridge.py` |

Quick start (from repo root):
```bash
# Deploy and launch robot processes:
./real_robot/scripts/deploy.sh --mode central --all

# Launch laptop processes (mocap + controller + control panel):
./real_robot/scripts/launch.sh --mode central
```

See [real_robot/README.md](real_robot/README.md) for full setup, pose
pipeline, controller selection, recording, and the sim→deploy pattern for
writing new controllers.

---

## Experiments

```bash
# MPC scaling study (solve time vs number of robots):
cd experiments/mpc_scaling
python3 run_scaling_experiment.py --n-values 2,4,8 --distance 5.0

# GBP distributed estimation demos:
cd experiments/gbp_estimation
python3 gbp_distributed_estimation.py
python3 test_async_dropout.py
```

---

## Demos

```bash
# Physics/contact validation:
cd demos/mujoco_swarm_demo
python3 scripts/demo.py

# Full research scenario:
cd demos/research_scenario_demo
python3 demo.py
```

---

## Notes for teammates

- After pulling: `git submodule sync && git submodule update --init models/holonomic_dp`
- The `swarmlib` venv install is editable (`pip install -e`), so local edits
  take effect immediately without reinstalling.
- Everything (`swarmlib`, `experiments`, `demos`, `real_robot`) is plain Python — no build step required.
