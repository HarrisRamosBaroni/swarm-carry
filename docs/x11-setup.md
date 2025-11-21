# X11 Display Setup (Linux/Ubuntu)

This guide covers setting up X11 forwarding for GUI applications (Gazebo, RViz) on Linux/Ubuntu systems. This provides native performance without browser-based access.

## Prerequisites

- Ubuntu/Linux host with X11 (most desktop installations)
- Docker installed

## Quick Setup

### 1. Allow Docker to Access Display

Before starting the container:

```bash
xhost +local:docker
```

This grants local Docker containers permission to access your X server.

### 2. Start Container with X11 Support

**Option A: Using devcontainer-x11.json (VSCode)**

Copy the X11 configuration:
```bash
cp .devcontainer/devcontainer-x11.json .devcontainer/devcontainer.json
```

Then open in VSCode and reopen in container.

**Option B: Using docker-run.sh script**

The script auto-detects X11 on Linux:
```bash
./scripts/docker-run.sh
```

**Option C: Manual docker run**

```bash
docker build -t swarm-sim .devcontainer/

docker run -it --rm \
    --volume="$(pwd):/workspace" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --env="DISPLAY=${DISPLAY}" \
    --network=host \
    --cap-add=SYS_PTRACE \
    --security-opt=seccomp:unconfined \
    --security-opt=apparmor:unconfined \
    --ipc=host \
    --workdir=/workspace \
    swarm-sim \
    /bin/bash
```

### 3. Launch GUI Application

Inside the container:

```bash
# Setup and build workspace
./scripts/setup.sh
export GZ_VERSION=harmonic
./scripts/build.sh
source install/setup.bash

# Launch simulation
./scripts/run.sh
```

Gazebo and other GUI windows will appear directly on your desktop!

## WSL2 with WSLg

Windows 11 with WSL2 includes WSLg (Windows Subsystem for Linux GUI), which provides X11 support.

### Setup

WSLg sets the DISPLAY variable automatically. Simply:

```bash
# Inside WSL2
echo $DISPLAY  # Should show something like :0

# Run container with X11
./scripts/docker-run.sh
```

GUI applications will appear as native Windows windows!

### Troubleshooting WSLg

If GUI doesn't appear:

1. Check DISPLAY variable:
   ```bash
   echo $DISPLAY
   ```

2. Update WSL:
   ```bash
   wsl --update
   ```

3. Restart WSL:
   ```powershell
   # In PowerShell
   wsl --shutdown
   ```

## How It Works

X11 forwarding works by:

1. **Sharing X11 socket**: `/tmp/.X11-unix` directory is mounted into container
2. **Setting DISPLAY**: Environment variable tells apps which display to use
3. **Permission grant**: `xhost +local:docker` allows Docker connections

The X server runs on the host, and containerized applications connect to it as clients.

## Security Note

`xhost +local:docker` only allows local Docker containers to access the display, not remote connections. This is generally safe for development.

To revoke access:
```bash
xhost -local:docker
```

## Comparison: X11 vs noVNC

| Feature | X11 | noVNC |
|---------|-----|-------|
| Platform | Linux, WSL2 | All (Windows/Mac/Linux) |
| Performance | Native (best) | Moderate (browser overhead) |
| Setup | One command | Browser access |
| GPU | Can use host GPU | Software rendering |
| Audio | Supported | Limited |
| Clipboard | Seamless | Manual copy/paste |

**Recommendation**: Use X11 on Linux/WSL2 for best performance. Use noVNC on Mac or if you prefer browser access.

## Advanced: GPU Acceleration

For GPU-accelerated rendering (Gazebo with Ogre2):

### NVIDIA GPU

```bash
# Install nvidia-container-toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Run container with GPU
docker run -it --rm \
    --gpus all \
    --volume="$(pwd):/workspace" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --env="DISPLAY=${DISPLAY}" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --network=host \
    swarm-sim
```

### Intel/AMD GPU

Intel/AMD GPUs work out-of-the-box with X11 forwarding. Add device access:

```bash
docker run -it --rm \
    --device=/dev/dri \
    --volume="$(pwd):/workspace" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --env="DISPLAY=${DISPLAY}" \
    --network=host \
    swarm-sim
```

## Troubleshooting

### "cannot open display"

1. Check DISPLAY is set inside container:
   ```bash
   echo $DISPLAY
   ```

2. Grant xhost access on host:
   ```bash
   xhost +local:docker
   ```

3. Verify X11 socket is mounted:
   ```bash
   ls /tmp/.X11-unix
   ```

### "No protocol specified"

Run on host:
```bash
xhost +local:docker
```

### GUI is very slow

1. Check if software rendering is forced:
   ```bash
   echo $LIBGL_ALWAYS_SOFTWARE  # Should be empty or 0
   ```

2. Enable GPU acceleration (see Advanced section)

3. Verify OpenGL:
   ```bash
   glxinfo | grep "OpenGL renderer"
   ```

### Permission denied on X11 socket

```bash
# On host
sudo chmod 777 /tmp/.X11-unix
xhost +local:docker
```

## Native Installation Alternative

If Docker complications persist, consider native ROS2 installation:

1. Install ROS2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
2. Install Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install
3. Clone repo and build:
   ```bash
   ./scripts/setup.sh
   ./scripts/build.sh
   source install/setup.bash
   ```

No X11 forwarding needed—everything runs natively!
