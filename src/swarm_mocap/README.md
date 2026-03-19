# swarm_mocap

Native ROS2 driver for the PhaseSpace MoCap system. No ROS1 or bridge required.

---

## How it works

The PhaseSpace hub is a networked device that streams tracking data over TCP.
`libowlsock.so` is PhaseSpace's own client SDK — it opens a socket directly to
the hub and receives a continuous event stream (LED positions, rigid body poses).
This node reads that stream, converts units (mm → m) and axes to ROS convention,
and publishes standard `geometry_msgs` types. ROS2 has no involvement until the
`publish()` call.

---

## Topics

| Topic | Type | Content |
|-------|------|---------|
| `/mocap/rigids` | `geometry_msgs/PoseArray` | All tracked rigid bodies per frame |
| `/mocap/markers` | `geometry_msgs/PoseArray` | All tracked LED markers per frame |
| `/mocap/rigid_{id}` | `geometry_msgs/PoseStamped` | Per rigid body (for IDs in `published_rigid_ids`) |

Rigids or markers with `cond ≤ 0` (not tracked) are silently dropped.

Coordinate convention matches the lab's existing setup:
`x = owl_x/1000`, `y = -owl_z/1000`, `z = owl_y/1000`.

---

## Running the driver

**One machine on the network runs the driver. Everyone else just subscribes.**

```bash
source /opt/ros/jazzy/setup.bash
cd src && colcon build --packages-select swarm_mocap --symlink-install
source install/setup.bash

ros2 launch swarm_mocap mocap.launch.py server_ip:=192.168.0.244
```

Before real experiments, set `published_rigid_ids` in `config/mocap_params.yaml`
to match the rigid body IDs assigned in the PhaseSpace web UI
(`http://192.168.0.244`) for each robot and the payload.

---

## Subscribing (any machine on the same ROS2 network)

```bash
ros2 topic echo /mocap/rigids
ros2 topic echo /mocap/rigid_0   # once published_rigid_ids is configured
```

Ensure all machines share the same `ROS_DOMAIN_ID` (default 0).

---

## Smoke test (no hardware)

```bash
source src/install/setup.bash
python3 src/swarm_mocap/test/smoke_test.py
```

Launches the node with a bad IP and confirms it fails cleanly.
