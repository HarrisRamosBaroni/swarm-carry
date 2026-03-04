# MuJoCo Multi-Robot Pushing Demo

A self-contained MuJoCo simulation package for demonstrating multi-robot object manipulation. Designed to validate MuJoCo's appropriateness for the swarm-carry project's multi-agent transportation research.

## Features

- **Configurable**: Easily adjust number of robots and objects
- **Multiple Scenarios**: Convergent pushing, collision testing, straight-line motion
- **Performance Metrics**: Real-time factor, contact statistics, object tracking
- **Simple Control**: Hardcoded trajectories demonstrate basic behaviors
- **Standalone**: No ROS dependencies, pure Python + MuJoCo

## Quick Start

### Prerequisites

```bash
# Install MuJoCo Python bindings
pip install mujoco

# Verify installation
python -c "import mujoco; print(mujoco.__version__)"
```

### Running the Demo

```bash
cd src/mujoco_swarm_demo/scripts

# Run default demo (2 robots, convergent push, 15s)
python run_demo.py

# Run collision test with 3 robots
python run_demo.py -s collision -r 3

# Quick straight-line test (8 seconds)
python run_demo.py -s straight -d 8
```

### Generating Custom Scenes

```bash
cd src/mujoco_swarm_demo/scripts

# Generate scene with 4 robots and 5 boxes
python generate_scene.py -r 4 -b 5 -o ../scenes/custom_scene.xml

# Run with custom scene
python run_demo.py --scene ../scenes/custom_scene.xml
```

## Demo Scenarios

### 1. Convergent Push (Default)
Robots converge on boxes from different sides and push collaboratively.

**Demonstrates:**
- Multi-agent coordination patterns
- Sustained contact dynamics
- Object displacement through pushing

**Duration:** 15 seconds

```bash
python run_demo.py -s convergent
```

### 2. Collision Test
Robots move in cyclic patterns causing frequent robot-robot and robot-object collisions.

**Demonstrates:**
- Contact stability
- Multi-body collision handling
- Physics realism

**Duration:** 15 seconds (or longer for stress testing)

```bash
python run_demo.py -s collision -d 30
```

### 3. Straight Lines
Simple forward motion until collision with objects.

**Demonstrates:**
- Basic contact behavior
- Physics response to impacts
- Simplest possible test case

**Duration:** 8 seconds

```bash
python run_demo.py -s straight -d 8
```

## Project Structure

```
mujoco_swarm_demo/
├── models/
│   ├── turtlebot3_waffle_pi.xml    # Robot model definition
│   └── assets/                      # STL meshes for visualization
│       ├── waffle_pi_base.stl
│       ├── left_tire.stl
│       ├── right_tire.stl
│       └── lds.stl
├── scenes/
│   └── scene.xml                    # Generated scene files
├── scripts/
│   ├── generate_scene.py            # Scene generation script
│   └── run_demo.py                  # Main demo controller
└── README.md                         # This file
```

## Command Line Options

### run_demo.py

```
python run_demo.py [OPTIONS]

Options:
  -s, --scenario {convergent,collision,straight}
                        Demo scenario to run (default: convergent)
  -d, --duration FLOAT  Simulation duration in seconds (default: 15.0)
  -r, --robots INT      Number of robots (default: 2)
  -b, --boxes INT       Number of boxes (default: 3)
  --scene PATH          Path to scene XML file (overrides generation)
```

### generate_scene.py

```
python generate_scene.py [OPTIONS]

Options:
  -r, --robots INT      Number of robots (default: 2)
  -b, --boxes INT       Number of boxes (default: 3)
  -o, --output PATH     Output XML file path (default: ../scenes/scene.xml)
```

## Performance Metrics

The demo automatically tracks and reports:

### Real-Time Performance
- **Real-Time Factor (RTF)**: Simulation speed vs real-time
  - RTF > 1.0: Faster than real-time
  - RTF = 1.0: Real-time
  - RTF < 1.0: Slower than real-time
- **Timestep**: Integration timestep (default: 2ms)

### Contact Statistics
- **Current Contacts**: Active contacts at each moment
- **Max Contacts**: Peak number of simultaneous contacts
- **Average Contacts**: Mean contacts over simulation

### Object Tracking
- **Box Displacement**: Distance each box moved from origin
- **Position Data**: XYZ coordinates (can be logged)

### Example Output

```
DEMO SUMMARY
============================================================
Simulation time:     15.00s
Real time:           12.34s
Real-time factor:    1.22x
Steps:               7500
Avg contacts:        8.3
Max contacts:        15
Timestep:            2.00ms
============================================================

Box displacement:
  Box 0: 1.234m from origin
  Box 1: 0.876m from origin
  Box 2: 0.543m from origin
```

## What to Report to Your Team

### 1. Contact Stability ✅
- **Observation**: MuJoCo maintains stable contacts during sustained pushing
- **Evidence**: Track contact count over time, should not oscillate wildly
- **Comparison Point**: Gazebo's ODE often has contact jitter issues

### 2. Physics Realism ✅
- **Friction Behavior**: Objects slide/stick appropriately
- **Collision Response**: Impulses look natural, no explosions or sinking
- **Multi-Body Dynamics**: Multiple robots can push same object without instability

### 3. Performance ✅
- **Expected RTF**: 0.5-2.0x on typical hardware (2-10 robots)
- **Scalability**: Test with increasing robot counts
- **Comparison**: Likely faster than Gazebo for contact-rich scenarios

### 4. Ease of Setup ✅
- **Scene Creation**: ~5 min to generate new configuration
- **Control Implementation**: Simple Python, direct actuator control
- **Learning Curve**: Moderate (XML knowledge helps, but not required)
- **Debugging**: Viewer is intuitive, easy to see what's happening

### 5. Suitability for Algorithms
- **Factor Graphs**: Can easily access all state data
- **Distributed Control**: Each robot has independent actuators
- **Contact Sensing**: Built-in contact detection
- **Custom Sensors**: Can add force, touch, IMU sensors

## Technical Details

### Robot Model
- **Type**: TurtleBot3 Waffle Pi (differential drive)
- **Actuators**: Velocity control on left/right wheels
  - Control range: -30 to +30 rad/s
- **Dynamics**: Free-floating base (6DOF), driven wheels

### Physics Settings
- **Integrator**: Implicit Fast (good for contacts)
- **Timestep**: 2ms (500 Hz)
- **Contact Parameters**:
  - Friction: (1.0, 0.005, 0.0001) - [sliding, torsional, rolling]
  - Solver: Soft contacts with compliance
  - Condim: 4 (friction cone with torsion)

### Objects
- **Boxes**: Various sizes (0.1m to 0.25m per side)
- **Masses**: 0.5kg to 2.0kg
- **Friction**: 0.7 (pushable but not too slippery)

## Troubleshooting

### "MuJoCo not found"
```bash
pip install mujoco
```

### Scene file not found
Run the scene generator first:
```bash
cd scripts && python generate_scene.py
```

### Simulation is too slow (RTF << 1.0)
- Reduce number of robots/boxes
- Increase timestep (edit XML: `<option timestep="0.005"/>`)
- Check CPU usage (MuJoCo is single-threaded for physics)

### Robots don't move
- Check that scene was generated correctly
- Verify actuator names match in XML and Python
- Try different scenario: `python run_demo.py -s straight`

### Viewer doesn't open
- Check X11/display settings (if using remote/Docker)
- MuJoCo viewer requires graphics support
- Try running on local machine, not via SSH

## Next Steps

See `.plan/mujoco_demo_proposal.md` for:
- Advanced demo ideas (coordinated transport, formation control)
- ROS 2 integration discussion
- Algorithm development roadmap
- Detailed comparison vs Gazebo

### Suggested Progression

1. ✅ **Run all three demos** - verify basic functionality
2. ✅ **Collect metrics** - note RTF, contact stats
3. **Compare to Gazebo** (optional) - same scenario in both simulators
4. **Implement advanced demo** - e.g., coordinated transport with goals
5. **Integrate algorithms** - factor graph methods from your papers

## Citation

Robot models from:
- **robotis_mujoco_menagerie**: TurtleBot3 models for MuJoCo
  - Source: ROBOTIS official MuJoCo Menagerie

MuJoCo:
- Todorov, E., Erez, T., & Tassa, Y. (2012). MuJoCo: A physics engine for model-based control.

## Contact

For questions about this demo package, refer to the main swarm-carry repository.

## License

Same as parent project (swarm-carry).
