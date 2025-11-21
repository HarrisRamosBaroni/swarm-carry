# ROS2 Gazebo Robotics Simulation

A ROS2 Jazzy + Gazebo Harmonic simulation template for robotics research and development. Currently contains an inverted pendulum control example; will be adapted for multi-agent payload transport.

## Quick Start

Choose your preferred workflow:

### Option A: Docker + X11 (Ubuntu/Linux - Fastest)

Best for Ubuntu/Linux users. GUI windows appear directly on your desktop.

```bash
# Start container (auto-detects X11)
./scripts/docker-run.sh

# Inside container: setup, build, run
./scripts/setup.sh
export GZ_VERSION=harmonic
./scripts/build.sh
source install/setup.bash
./scripts/run.sh
```

See [docs/x11-setup.md](docs/x11-setup.md) for details.

### Option B: Docker + VNC (Windows/Mac/Linux)

Works on all platforms. Connect with VNC client or browser.

```bash
# 1. Start container
./scripts/docker-run.sh

# 2. Inside container, start VNC server
./scripts/start-vnc.sh

# 3. Connect with VNC client (TigerVNC, RealVNC, Remmina)
#    Host: localhost:5901
#    Password: vncpass

# OR use browser (noVNC):
./scripts/start-novnc.sh
# Then open: http://localhost:6080

# 4. In VNC desktop or separate terminal: setup and run
./scripts/setup.sh
export GZ_VERSION=harmonic
./scripts/build.sh
source install/setup.bash
./scripts/run.sh
```

See [docs/vnc-setup.md](docs/vnc-setup.md) for client installation and details.

### Option C: VSCode Dev Container (Optional)

If you prefer VSCode integration:

```bash
# 1. Open folder in VSCode
# 2. Command palette: "Dev Containers: Reopen in Container"
# 3. Start VNC: ./scripts/start-vnc.sh
# 4. Connect with VNC client or browser
# 5. Run scripts as above
```

See [docs/vscode-setup.md](docs/vscode-setup.md) for details.

### Option D: Native Installation

Install ROS2 and Gazebo directly on your system (no containers).

```bash
# 1. Install ROS2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
# 2. Install Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install
# 3. Clone this repo and build:
./scripts/setup.sh
export GZ_VERSION=harmonic
./scripts/build.sh
source install/setup.bash

# 4. Run simulation
./scripts/run.sh
```

## Project Structure

```
├── .devcontainer/          # Docker container configuration
│   ├── Dockerfile          # Container image definition
│   ├── devcontainer.json   # Default (noVNC)
│   ├── devcontainer-x11.json     # X11 variant (Linux)
│   └── devcontainer-novnc.json   # noVNC variant (all platforms)
│
├── docs/                   # Documentation
│   ├── x11-setup.md        # X11 display forwarding (Linux)
│   ├── novnc-setup.md      # Browser-based GUI access
│   ├── vscode-setup.md     # VSCode integration (optional)
│   └── architecture.md     # System technical details
│
├── scripts/                # Workflow scripts
│   ├── setup.sh            # Install dependencies
│   ├── build.sh            # Build ROS2 workspace
│   ├── run.sh              # Launch simulation
│   ├── docker-run.sh       # Start Docker container
│   ├── docker-shell.sh     # Open shell in running container
│   ├── start-vnc.sh        # Start VNC server (in container)
│   ├── start-novnc.sh      # Start noVNC web interface
│   └── stop-vnc.sh         # Stop VNC services
│
└── src/                    # ROS2 workspace source
    ├── inverted_pendulum/  # Example packages (to be adapted)
    │   ├── inverted_pendulum_description/   # Robot models (URDF/SDF)
    │   ├── inverted_pendulum_gazebo/        # Gazebo worlds and config
    │   ├── inverted_pendulum_controller/    # Control algorithms
    │   └── inverted_pendulum_bringup/       # Launch files
    │
    └── ros2.repos          # External dependencies
```

## Development Workflow

### First Time Setup

1. **Choose your environment** (Docker or native)
2. **Install dependencies**: `./scripts/setup.sh`
3. **Set Gazebo version**: `export GZ_VERSION=harmonic` (add to `~/.bashrc` to persist)
4. **Build workspace**: `./scripts/build.sh`
5. **Source workspace**: `source install/setup.bash` (or add to `~/.bashrc`)

### Making Changes

```bash
# 1. Edit code in src/
# 2. Rebuild
./scripts/build.sh

# 3. Source (if new packages or launch files)
source install/setup.bash

# 4. Test
./scripts/run.sh
```

**Note**: With `--symlink-install`, Python changes don't need rebuild. C++ changes do.

### Build Types

```bash
# Release (optimized, default)
./scripts/build.sh

# Debug (with symbols)
BUILD_TYPE=Debug ./scripts/build.sh

# Release with debug info
BUILD_TYPE=RelWithDebInfo ./scripts/build.sh
```

## Current Example: Inverted Pendulum

The repo currently contains an inverted pendulum on cart simulation with control. This demonstrates:
- ROS2 + Gazebo integration
- ros2_control framework
- URDF/SDF robot modeling
- Launch file composition

### Running the Example

```bash
# After setup and build
source install/setup.bash
export GZ_VERSION=harmonic
ros2 launch inverted_pendulum_bringup inverted_pendulum.launch.py

# GUI access:
# - X11: Windows appear on desktop
# - noVNC: Open http://localhost:6080
```

**Important**: After stopping (Ctrl+C), wait 30 seconds before restarting to allow clean shutdown.

### System Architecture

See [docs/architecture.md](docs/architecture.md) for detailed explanation of:
- Package organization
- Data flow (Gazebo → ROS2 → Controller → Gazebo)
- ros2_control integration
- Configuration files

## Future Development: Multi-Agent Payload Transport

This repo will be adapted for research on distributed multi-agent payload transport. Planned features:
- Multiple robot agents (ground/aerial)
- Payload attachment and manipulation
- Distributed control algorithms
- Formation control and planning

The current structure serves as a template demonstrating ROS2/Gazebo best practices.

## Environment Configuration

Copy `.env.example` to `.env` and customize:

```bash
cp .env.example .env
# Edit .env with your preferences
```

Variables:
- `ROS_DISTRO`: ROS2 version (default: jazzy)
- `GZ_VERSION`: Gazebo version (default: harmonic)
- `BUILD_TYPE`: Compilation mode (Release/Debug/RelWithDebInfo)

## GUI Options Comparison

| Method | Platform | Performance | Client Installation |
|--------|----------|-------------|---------------------|
| **X11** | Linux, WSL2 | Excellent | None (built-in) |
| **VNC Client** | All | Good | One-time (TigerVNC, etc.) |
| **noVNC (Browser)** | All | Good | None |
| **Native** | All | Excellent | ROS2 + Gazebo install |

**Recommendation**:
- **Ubuntu/Linux**: Use X11 (fastest, zero setup)
- **Windows/Mac**: Use VNC client (install once) or noVNC (browser)
- **Serious development**: Consider native installation

## Tips

### VSCode Integration (Optional)

VSCode is completely optional but provides nice features:
- Integrated terminal with auto-sourcing
- Task shortcuts (Ctrl+Shift+B to build)
- IntelliSense for C++/Python
- ROS2 debugging

See [docs/vscode-setup.md](docs/vscode-setup.md) to set up.

### Multiple Terminals

**Docker container access**:
```bash
# Open additional shell in running container
./scripts/docker-shell.sh
```

**VSCode**: Just open new integrated terminal (automatically inside container)

### Gazebo Performance

If Gazebo is slow:
1. Using software rendering (Docker): This is normal, consider native install
2. Enable GPU acceleration: See [docs/x11-setup.md](docs/x11-setup.md) GPU section
3. Lower physics update rate in world file

### Clean Build

```bash
# Remove all build artifacts
rm -rf build/ install/ log/

# Or use sudo if permission issues
sudo rm -rf build/ install/ log/
sudo py3clean .

# Then rebuild
./scripts/build.sh
```

## Documentation

- **[docs/x11-setup.md](docs/x11-setup.md)**: X11 display forwarding for Linux
- **[docs/vnc-setup.md](docs/vnc-setup.md)**: VNC client and noVNC browser access
- **[docs/vscode-setup.md](docs/vscode-setup.md)**: VSCode integration (optional)
- **[docs/architecture.md](docs/architecture.md)**: Technical system overview

External:
- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
- [ros2_control](https://control.ros.org/)

## Troubleshooting

### "Cannot find package"

```bash
# Install dependencies
./scripts/setup.sh

# Rebuild and source
./scripts/build.sh
source install/setup.bash
```

### "Multiple controller managers"

Previous simulation didn't clean up. Wait 30 seconds after Ctrl+C before restarting.

### "Cannot open display" (Docker)

**X11 mode**: Script handles this automatically

**VNC mode**: Start VNC first with `./scripts/start-vnc.sh`

See platform-specific docs for details.

### GUI is slow/laggy

- **Docker**: Normal with software rendering. Use native install or GPU forwarding.
- **noVNC**: Lower resolution or use X11 on Linux.

## Contributing

This is a research project template. When adapting for multi-agent work:

1. Create new packages in `src/` (not under `inverted_pendulum/`)
2. Follow naming convention: `<project>_description`, `<project>_control`, etc.
3. Update this README with new launch commands
4. Keep documentation up to date

## License

See [LICENSE](LICENSE) file.

## Acknowledgments

Based on [athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace) template.

Adapted for ROS2 Jazzy + Gazebo Harmonic with simplified onboarding.
