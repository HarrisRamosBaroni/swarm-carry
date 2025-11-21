# System Architecture

Technical overview of the ROS2 + Gazebo + ros2_control integration.

## Overview

This project demonstrates a complete robotics simulation stack using:
- **ROS2 Jazzy**: Robot middleware and communication
- **Gazebo Harmonic**: Physics simulation
- **ros2_control**: Real-time control framework

Currently contains an inverted pendulum example that will be adapted for multi-agent payload transport.

## Package Structure

### Current Packages (Inverted Pendulum Example)

Located in `src/inverted_pendulum/`:

#### `inverted_pendulum_description`
- **Purpose**: Robot model definitions
- **Contents**:
  - `models/inverted_pendulum/urdf/model.urdf` - Robot URDF description
  - Asset files (meshes, textures)
- **Key plugin**: `gz_ros2_control-system` - Bridges Gazebo and ros2_control

#### `inverted_pendulum_gazebo`
- **Purpose**: Gazebo-specific code and configurations
- **Contents**:
  - `worlds/` - Gazebo world SDF files
  - Gazebo plugins (if any)

#### `inverted_pendulum_controller`
- **Purpose**: High-level control logic
- **Contents**:
  - `config/cart_controllers.yaml` - ros2_control controller configuration
  - Python/C++ control nodes
- **Note**: Publishes commands to ros2_control controllers

#### `inverted_pendulum_bringup`
- **Purpose**: System integration and launch
- **Contents**:
  - `launch/inverted_pendulum.launch.py` - Main launch file
  - High-level configuration files

## System Integration

### Element Relationships

```
┌─────────────────┐
│  Bringup Pkg    │  Launches everything
└────────┬────────┘
         │
    ┌────┴──────┬─────────┬────────────┐
    ▼           ▼         ▼            ▼
┌────────┐  ┌──────┐  ┌────────┐  ┌─────────┐
│Gazebo  │  │Robot │  │Control │  │ros2     │
│Sim     │  │State │  │Node    │  │control  │
│        │  │Pub.  │  │        │  │spawners │
└────────┘  └──────┘  └────────┘  └─────────┘
```

### Data Flow

```
1. URDF loaded in Gazebo
   ↓
2. gz_ros2_control plugin starts controller_manager
   ↓
3. Gazebo publishes joint states
   │
   ▼ (via gz_ros2_control)
4. /joint_states (ROS2 topic)
   │
   ▼ (subscribed by control node)
5. Control algorithm computes command
   │
   ▼ (publishes to)
6. /cart_effort_controller/commands
   │
   ▼ (ros2_control controller receives)
7. Controller applies force in Gazebo
   │
   ▼
8. Physics engine updates simulation
   │
   └──▶ Back to step 3 (loop)
```

## Launch Sequence

The `inverted_pendulum.launch.py` script orchestrates:

1. **Start Gazebo Sim**
   - Loads physics engine
   - Initializes rendering

2. **Controller Manager Starts** (automatic)
   - Plugin `gz_ros2_control-system` in URDF triggers this
   - Reads configuration from `cart_controllers.yaml`

3. **Load World**
   - Spawns environment (ground plane, lighting, etc.)

4. **Spawn Robot**
   - Inserts URDF model into world
   - Establishes Gazebo ↔ ROS2 bridge

5. **Start robot_state_publisher**
   - Publishes robot description to `/robot_description` topic
   - Controller manager uses this to configure controllers

6. **Start Control Node**
   - User-defined control logic
   - Subscribes to sensor data
   - Publishes control commands

7. **Spawn ros2_control Controllers**
   - Effort controller, joint state broadcaster, etc.
   - Connect to hardware interfaces (Gazebo simulation in this case)

8. **Start Topic Bridges**
   - `inverted_pendulum_bridge.yaml` defines topic mappings
   - Converts Gazebo topics ↔ ROS2 topics

9. **Optional: RViz**
   - Visualization tool
   - Displays robot model and sensor data

## Configuration Files

### cart_controllers.yaml

Defines ros2_control controllers:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

cart_effort_controller:
  type: effort_controllers/JointGroupEffortController
  joints:
    - cart_joint
```

### inverted_pendulum_bridge.yaml

Maps Gazebo topics to ROS2:

```yaml
- topic_name: /joint_states
  gz_topic_name: /world/demo/model/inverted_pendulum/joint_state
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
```

## Key Concepts

### gz_ros2_control Plugin

The URDF contains:

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find inverted_pendulum_controller)/config/cart_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

This:
1. Starts a ros2_control controller manager inside Gazebo
2. Creates hardware interface for simulated robot
3. Allows ros2_control controllers to interact with Gazebo physics

### Controller Manager

Central hub for ros2_control:
- Loads controllers from configuration
- Manages controller lifecycle (start/stop)
- Routes commands to hardware interfaces

### Hardware Interface

Abstraction layer between controllers and hardware:
- **Simulation**: Gazebo provides interface
- **Real robot**: Custom interface talks to motor drivers

Same controllers work for both!

## Future: Multi-Agent Architecture

Planned structure for payload transport:

```
src/
├── swarm_description/     # Multi-agent models
├── swarm_gazebo/          # Multi-robot world
├── swarm_control/         # Distributed controllers
├── swarm_planning/        # Formation, path planning
├── swarm_msgs/            # Custom message types
└── swarm_bringup/         # Launch system
```

Key changes:
- Multiple robot instances
- Inter-agent communication
- Distributed control algorithms
- Payload attachment models

## References

- [ROS2 Control Documentation](https://control.ros.org/)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
- [gz_ros2_control GitHub](https://github.com/ros-controls/gz_ros2_control)

## Troubleshooting

### Multiple controller managers spawn

**Symptom**: Duplicate nodes, conflicting controllers

**Cause**: Previous simulation not fully cleaned up

**Solution**: Wait 30 seconds after Ctrl+C before restarting

### Controller not loading

**Symptom**: `cart_effort_controller` not found

**Cause**: Configuration file not installed or sourced

**Solution**:
```bash
# Rebuild with --symlink-install
./scripts/build.sh
source install/setup.bash
```

### Joint states not publishing

**Symptom**: No data on `/joint_states`

**Cause**: Bridge not running or misconfigured

**Solution**: Check `inverted_pendulum_bridge.yaml` mapping
