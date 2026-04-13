# real_robot

Real-robot deployment using ZeroMQ for cross-machine comms and ROS1 locally on each myAGV. No ROS2 on the robots.

---

## Prerequisites

**Laptop** (WSL/Ubuntu 24.04): `pip install pyzmq msgpack pyyaml`. And `pip install rclpy` unless you want to source package from ros2 install.

**Each myAGV** (Ubuntu 18.04 / ROS1 Melodic):
```bash
pip install pyzmq msgpack pyyaml qwiic_nau7802 qwiic_i2c
```
Also needs `myagv_ros` already installed and working.

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

### Laptop
```bash
# Terminal 1 — mocap
source /opt/ros/jazzy/setup.bash && source src/install/setup.bash
ros2 launch swarm_mocap mocap.launch.py server_ip:=192.168.0.244

# Terminal 2 — mocap → ZeroMQ bridge
python real_robot/laptop/mocap_bridge.py --config real_robot/config/network.yaml

# Terminal 3 — centralised controller (skip for decentralised)
python real_robot/laptop/central_runner.py --config real_robot/config/network.yaml --n-robots 2 --goal 5.0 0.0 0.0
```

### Each myAGV (SSH in)
```bash
# Terminal 1 — local ROS1 stack
roslaunch myagv_ros myagv_active.launch

# Terminal 2 — agent
python real_robot/robot/agent_runner.py \
    --config /home/ubuntu/network.yaml \
    --id 0 \
    --neighbors 1 \
    --goal 5.0 0.0 0.0
```
Change `--id` and `--neighbors` per robot. For decentralised mode the laptop terminal 3 is not needed.

---

## Plugging in a controller

Open `real_robot/laptop/central_runner.py` (centralised) or `real_robot/robot/agent_runner.py` (decentralised) and replace the `self.controller = None` line:

```python
from swarmlib.controllers import YourController
self.controller = YourController(num_robots=n_robots, ...)
```

The controller must implement `compute_control(payload_state, robot_states, goal_state, dt, forces)` — same interface as the simulation.

---

## Load cell config path

`agent_runner.py` constructs `LoadCellReader(config_path=...)` — update that path to wherever you put the calibrated `config.yaml` on the robot (default is `"config.yaml"` in the working directory).
