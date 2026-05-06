# real_robot

Real-robot deployment using ZeroMQ for cross-machine comms and ROS1 locally on each myAGV. No ROS2 on the robots.

---

## Prerequisites

**Laptop** (WSL/Ubuntu 24.04): `pip install pyzmq msgpack pyyaml`. No ROS2 required.

**Each myAGV** (Ubuntu 18.04 / ROS1 Melodic):
```bash
pip install pyzmq msgpack pyyaml qwiic_nau7802 qwiic_i2c dataclasses gtsam
```
Also needs `myagv_ros` already installed and working.

---

## Pose pipeline

All ground-truth poses flow from a single source — the PhaseSpace server — and are fanned out over ZMQ by `mocap_bridge`. No robot touches the mocap server directly (the OWL binary is x86-only).

```
PhaseSpace server
    │  OWL (TCP, mm, PhaseSpace axes)
    ▼
mocap_pub.py (laptop)               connects via libowlsock directly (no ROS2)
    │                               converts mm→m, swaps axes
    │                               maps PhaseSpace IDs → application IDs
    │                               robots: phasespace_id → robot id (0,1,2…)
    │                               payload: phasespace_id → id -1 (sentinel)
    │  ZMQ PUB "pose" {id, x, y, theta}
    ▼
central_runner / agent_runner       consume by id; robots differentiate
                                    consecutive poses for vx, vy
```

Build once (from repo root):
```bash
cd real_robot/scripts && make && cd -
```

PhaseSpace rigid body IDs are defined in the PhaseSpace web UI and mapped to application IDs in `network.yaml` via the `mocap_rigid_id` field on each robot entry and the `payload.mocap_rigid_id` field.

---

## One-time setup

**1. Fill in IPs** — edit `real_robot/config/network.yaml` with the actual IPs of each robot and the laptop.

**2. Calibrate load cells** (run once per robot on the myAGV):
```bash
python swarmlib/sensors/force/calibrate_vals.py
```
Copy the printed `zeroOffset` / `calFactor` values into a `config.yaml` (see `swarmlib/sensors/force/config.yaml.example`). Put that file somewhere accessible on the robot, e.g. `/home/ubuntu/force_config.yaml`.

**3. Copy network config to each robot:**
```bash
scp real_robot/config/network.yaml ubuntu@192.168.0.101:/home/ubuntu/network.yaml
```

---

## Run order

Two scripts handle the full launch. Run both from the repo root.

**Robots** (`deploy.sh` — handles yaml sync, git pull, tmux sessions on each robot):
```bash
# centralised: robots are passive, central_runner drives them
./real_robot/scripts/deploy.sh --mode central --all

# decentralised: robots self-drive; --neighbors computed automatically per robot
./real_robot/scripts/deploy.sh --mode decentralised --all
```

Flags can be combined freely: `--yaml`, `--pull`, `--launch` (or `--all`). Extra agent args append via env: `AGENT_EXTRA_ARGS="--gbp-async" ./deploy.sh --mode decentralised --launch`.

**Laptop** (`launch.sh` — opens a local tmux session `swarm-laptop`):
```bash
# centralised: mocap window + controller window
./real_robot/scripts/launch.sh --mode central --goal 2 0 0

# decentralised: mocap window only
./real_robot/scripts/launch.sh --mode decentralised
```

Attach anytime: `tmux attach -t swarm-laptop`. Kill: `tmux kill-session -t swarm-laptop`.

Central-mode options: `--n-robots N`, `--gt-payload`, `--relative-goal`, `--viewer`, `--server IP`.

To monitor live poses:
```bash
python3 -m real_robot.scripts.mocap_echo
```

---

## Plugging in a controller

### Centralised

`central_runner.py` ships wired to `MRCapController`. Two payload-pose modes:

- **Estimator mode (default)** — no payload mocap rigid body required. The runner synthesises an init payload pose from initial robot positions; `CentroidEstimator` calibrates body-frame offsets `r_i` once and infers payload pose+vel from robot states thereafter. Smoke-test friendly.
- **GT mocap mode** — pass `--gt-payload`. Requires a `payload` rigid body in the mocap software (registered under `payload.mocap_rigid_id` in `network.yaml`). `mocap_bridge.py` forwards it with sentinel id `-1`.

To swap controllers, edit the `MRCapController(...)` instantiation in `central_runner.py`. Any controller with the standard `compute_control(payload_state, robot_states, goal_state, dt, forces)` signature is a drop-in replacement.

### Decentralised — sim→deployment pattern

Decentralised controllers in this repo follow one pattern so that the same class runs unchanged in sim and on real robots. `DRCapDistributedController` is the reference implementation (`swarmlib/controllers/drcap_distributed_controller.py`). Any new decentralised controller the team writes should follow the same pattern:

**1. Accept `my_id` and `backend` as constructor args.**
```python
YourController(
    num_robots=N,
    formation=formation,
    backend=backend,      # injected — do NOT construct inside the class
    my_id=None,           # None = sim (manage all N local graphs); int = deploy (only mine)
    topology=topology,
    config={...},
)
```

**2. Two behaviours in `compute_control` based on `my_id`:**

| Mode | `my_id` | `robot_states` shape | Return shape | Backend |
|---|---|---|---|---|
| Simulation | `None` | `(N, 4)` | `(N, 2)` | `SimulatedBackend` / `AsyncSimulatedBackend` (one process drives all agents) |
| Deployment | `int` | `(1, 4)` — this robot only | `(1, 2)` — this robot's command | `ZeroMQSingleAgentBackend` (one process per robot) |

Internally, maintain a dict of local graphs keyed by robot id. In sim mode build all `N`; in deploy mode build only `my_id`. The GBP / message loop iterates over owned ids — the code is identical in both modes, just the set of owned ids differs.

**3. Experiment runner vs deployment runner.**
- Sim: `experiments/<your_method>/run_experiment.py` instantiates the controller with `my_id=None` and a `SimulatedBackend` / `AsyncSimulatedBackend` (see `experiments/drcap_fg/run_experiment.py`).
- Deploy: `real_robot/robot/agent_runner.py` instantiates one controller per robot process with `my_id=robot_id` and the ZMQ backend it already builds (see how `DRCapDistributedController` is wired in that file). To adopt for your own controller, just change the import + constructor call — no other runner changes needed.

**4. Sync vs async GBP.** `ZeroMQSingleAgentBackend(..., synchronous=False)` makes `barrier()` non-blocking — each agent iterates at its own pace using whatever neighbor beliefs have arrived. Pass `--gbp-async` to `agent_runner.py` to enable this. The paper's DR.CAP evaluations use asynchronous GBP; controller code does not need to change between sync and async.

Testing order: sim (`SimulatedBackend`) → sim with dropout (`AsyncSimulatedBackend`) → deployment (`ZeroMQSingleAgentBackend`, sync then async). Each step only changes the backend + `my_id`.

---

## Load cell config path

`agent_runner.py` constructs `LoadCellReader(config_path=...)` — update that path to wherever you put the calibrated `config.yaml` on the robot (default is `"config.yaml"` in the working directory).
