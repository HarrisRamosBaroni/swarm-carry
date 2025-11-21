# VNC GUI Access Setup

This guide covers VNC-based GUI access for Gazebo and RViz. Works on all platforms (Windows, Mac, Linux). Two options: VNC client (recommended) or browser (noVNC).

## What You Get

The Docker image includes:
- **TigerVNC server** - VNC server running in container
- **XFCE desktop** - Lightweight desktop environment
- **noVNC** - Optional web interface (no client install needed)

## Quick Start

```bash
# 1. Start container
./scripts/docker-run.sh

# 2. Inside container, start VNC server
./scripts/start-vnc.sh

# 3a. Connect with VNC client (recommended)
#     - Host: localhost:5901
#     - Password: vncpass
#     - Clients: TigerVNC, RealVNC, Remmina

# 3b. OR use browser (noVNC)
./scripts/start-novnc.sh
# Then open: http://localhost:6080
```

## Option 1: VNC Client (Recommended)

**Why VNC client?**
- Better performance than browser
- Native clipboard support
- One-time install

**Install a VNC client:**

| Platform | Recommended Client | Install |
|----------|-------------------|---------|
| **Linux** | Remmina | `sudo apt install remmina` |
| **Windows** | TigerVNC | [tigervnc.org](https://tigervnc.org/) |
| **Mac** | TigerVNC | [tigervnc.org](https://tigervnc.org/) |

**Connect:**
1. Start VNC in container: `./scripts/start-vnc.sh`
2. Open VNC client on host
3. Connect to: `localhost:5901`
4. Password: `vncpass`
5. Desktop appears!

## Option 2: Browser (noVNC)

No client installation needed.

```bash
# Inside container
./scripts/start-vnc.sh      # Start VNC first
./scripts/start-novnc.sh    # Start web interface

# Open browser to: http://localhost:6080
# Password: vncpass
```

## Using the VNC Desktop

### Open Terminal in VNC

- Applications menu (top left) → Terminal Emulator
- Or right-click desktop → Open Terminal Here

### Run Simulation

```bash
cd /workspace
source install/setup.bash
export GZ_VERSION=harmonic
./scripts/run.sh
```

Gazebo opens in VNC desktop!

### Multiple Terminals

- **In VNC**: Open multiple terminal windows from Applications menu
- **From host**: Run `./scripts/docker-shell.sh` for additional shells

## Managing VNC

```bash
# Start VNC
./scripts/start-vnc.sh

# Start noVNC (optional)
./scripts/start-novnc.sh

# Stop all VNC services
./scripts/stop-vnc.sh

# Custom resolution
VNC_RESOLUTION=1280x720 ./scripts/start-vnc.sh
```

## VNC Client Setup Details

### Remmina (Linux)

```bash
# Install
sudo apt install remmina remmina-plugin-vnc

# Launch and connect
# Protocol: VNC
# Server: localhost:5901
# Password: vncpass
```

### TigerVNC (Windows/Mac/Linux)

1. Download from [tigervnc.org](https://tigervnc.org/)
2. Install and launch viewer
3. VNC Server: `localhost:5901`
4. Password: `vncpass`

### macOS Screen Sharing

1. Finder → Go → Connect to Server (Cmd+K)
2. Server: `vnc://localhost:5901`
3. Password: `vncpass`

## Comparison

| Feature | VNC Client | noVNC (Browser) |
|---------|-----------|-----------------|
| Installation | One-time | None |
| Performance | Better | Good |
| Clipboard | Native | Manual panel |
| Best for | Regular use | Quick demos |

**Recommendation**: Install VNC client for daily work, use noVNC as fallback.

## Troubleshooting

### "Cannot connect"

```bash
# Check VNC is running
pgrep -x Xtigervnc

# Restart if needed
./scripts/stop-vnc.sh
./scripts/start-vnc.sh
```

### Slow performance

```bash
# Lower resolution
VNC_RESOLUTION=1280x720 ./scripts/start-vnc.sh

# Disable compositing in XFCE
# Settings → Window Manager Tweaks → Compositor → Disable
```

### Port already in use

```bash
# Use different port
VNC_DISPLAY=2 ./scripts/start-vnc.sh  # Uses port 5902
```

## Security Note

Default password is `vncpass` - fine for local development.

To change: Edit `.devcontainer/Dockerfile` and rebuild.

## Linux Users: Consider X11

If you're on Linux/WSL2, X11 is faster than VNC. See [docs/x11-setup.md](x11-setup.md).

X11 gives native performance with zero client installation!
