# HOWTO: Swarm Transport Infrastructure

Quick reference for running, extending, and deploying the swarm transport stack.

---

## 1. Run the sim with any controller

```python
import numpy as np
from simulation import SwarmTransportEnv

env = SwarmTransportEnv(n_robots=4, goal_pos=[5.0, 0.0, 0.0])
obs = env.reset()

for _ in range(500):
    # obs keys: 'payload' (6,), 'robots' (n,4), 'forces' (n,3)
    controls = np.zeros((4, 2))      # [vx, vy] per robot, m/s
    obs = env.step(controls)
    print(f"t={env.time:.2f}  payload_x={obs['payload'][0]:.3f}")

env.close()
```

Run the full MPC scaling experiment:
```bash
cd src/control_scaling_experiments/experiments
python run_scaling_experiment.py --n 2,4 --distance 5.0
```

---

## 2. Write a new controller

Implement `BaseController`. The only requirement: `compute_control` returns
`(n, 2)` Cartesian `[vx, vy]` in m/s (world frame). The env handles
diff-drive conversion internally.

```python
import numpy as np
from controllers.base_controller import BaseController

class ZeroController(BaseController):
    def compute_control(self, payload_state, robot_states, goal_state,
                        dt, forces=None):
        return np.zeros((self.num_robots, 2))

    def reset(self):
        pass
```

Then use it with `SwarmTransportEnv`:
```python
env = SwarmTransportEnv(n_robots=2)
ctrl = ZeroController(num_robots=2)
obs = env.reset()
ctrl.reset()
while env.time < 10.0:
    obs = env.step(ctrl.compute_control(
        obs['payload'], obs['robots'], env.goal_pos, env.dt, obs['forces']
    ))
```

The utility `cartesian_to_diff_drive(vx, vy, heading_rad)` is also exported
from `controllers.base_controller` if you need it in your own code.

---

## 3. Use ROS2 backend for algorithm communication

The `CommunicationBackend` handles **peer-to-peer algorithm messages** (GBP
beliefs, factor graph messages, etc.). It is independent of the physics bridge.

```python
from communication.backend import SimulatedBackend, create_ring_topology
from communication.ros2_backend import ROS2Backend

# For testing (no ROS2 needed):
topo = create_ring_topology(4)
backend = SimulatedBackend(num_agents=4, topology=topo)

# For real networked deployment (requires ROS2):
backend = ROS2Backend(num_agents=4, topology=topo)

# API is identical either way:
backend.broadcast(from_id=0, message=my_gaussian_msg)
backend.barrier()
messages = backend.receive(agent_id=1)
```

---

## 4. Full ROS2 simulation (bridge + controller node)

**Terminal 1 — start the physics bridge:**
```bash
source /opt/ros/humble/setup.bash
cd src/swarm_mujoco_bridge
colcon build --symlink-install
source install/setup.bash
ros2 launch swarm_mujoco_bridge sim.launch.py n_robots:=2 push_distance:=5.0
```

**Verify it's publishing:**
```bash
ros2 topic echo /swarm/payload/state
ros2 topic echo /swarm/robot_0/force
```

**Terminal 2 — run centralized controller:**
```bash
# Edit swarm_central/swarm_central/central_controller_node.py:
#   self.controller = CentralizedMPC(num_robots=2, config={...})
ros2 launch swarm_central central.launch.py n_robots:=2
```

---

## 5. Real robot deployment (TurtleBot3)

**Architecture:**

| Machine | Process | Notes |
|---------|---------|-------|
| Laptop | `swarm_mujoco_bridge` *or* nothing | Use bridge for sim, skip for real robots |
| Each TurtleBot3 | `swarm_agent agent_node` | One per robot; reads state from onboard sensors or bridge |
| Laptop *or* any machine | `swarm_central central_node` | Centralized methods only |

**On each TurtleBot3:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch swarm_agent agent.launch.py \
    agent_id:=0 neighbor_ids:="1,2" goal_x:=5.0
```

**On laptop (centralized method):**
```bash
ros2 launch swarm_central central.launch.py n_robots:=3 goal_x:=5.0
```

**On laptop (decentralized method):** each robot's `agent_node` runs its
own controller; `SingleAgentROS2Backend` handles GBP peer messages via
`/swarm/agent_{i}/outbox` topics.

**Topic map (same for sim and real):**

```
/swarm/payload/state        ← bridge (sim) or payload sensor node (real)
/swarm/robot_{i}/state      ← bridge (sim) or robot odometry node (real)
/swarm/robot_{i}/force      ← bridge (sim) or F/T sensor node (real)
/swarm/robot_{i}/cmd_vel    → bridge (sim) applies to MuJoCo actuators
                               real robot: forward to motor controller
/swarm/agent_{i}/outbox     ← GBP algorithm messages (peer-to-peer)
```

Controller nodes do not know whether they talk to sim or hardware —
the topic interface is identical.
