#!/bin/bash
# Direct Docker run (alternative to VSCode devcontainer)
# Supports X11 (Linux) or VNC/noVNC (all platforms)

set -e

IMAGE_NAME="swarm-sim"
CONTAINER_NAME="swarm-sim-dev"

# Detect if we should use X11
USE_X11=false
if [ -n "$DISPLAY" ] && [ "$(uname)" == "Linux" ] && [ -d "/tmp/.X11-unix" ]; then
    # Check if we can actually mount this directory with Docker
    if docker run --rm -v /tmp/.X11-unix:/tmp/.X11-unix alpine true 2>/dev/null; then
        USE_X11=true
    else
        echo "Warning: Cannot use X11 forwarding."
        if [ "$XDG_SESSION_TYPE" = "wayland" ]; then
            echo "Detected Wayland session - X11 socket sharing blocked by Docker Desktop."
        else
            echo "X11 socket exists but Docker cannot access it."
        fi
        echo "Falling back to VNC mode (recommended for Wayland)."
        echo ""
    fi
fi

# Build the image if it doesn't exist
if ! docker image inspect "$IMAGE_NAME" &> /dev/null; then
    echo "Building Docker image '$IMAGE_NAME'..."
    echo "(This includes VNC server - takes ~5 minutes first time)"
    docker build -t "$IMAGE_NAME" .devcontainer/
fi

# Remove existing container if present
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container '$CONTAINER_NAME'..."
    docker rm -f "$CONTAINER_NAME"
fi

# Common Docker arguments
DOCKER_ARGS=(
    -it
    --name "$CONTAINER_NAME"
    --volume="$(pwd):/workspace"
    --cap-add=SYS_PTRACE
    --security-opt=seccomp:unconfined
    --security-opt=apparmor:unconfined
    --ipc=host
    --workdir=/workspace
    --user=ros
)

# Set up X11 or VNC
if [ "$USE_X11" = true ]; then
    echo "=== X11 Mode (Linux) ==="
    echo "GUI windows will appear on your desktop"
    xhost +local:docker

    docker run "${DOCKER_ARGS[@]}" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
        --env="DISPLAY=${DISPLAY}" \
        --network=host \
        "$IMAGE_NAME" \
        /bin/bash

    # Clean up X11 permissions on exit
    xhost -local:docker
else
    echo "=== VNC Mode ==="
    echo "VNC server is installed in container"
    echo ""
    echo "After container starts:"
    echo "  1. Start VNC: ./scripts/start-vnc.sh"
    echo "  2. Connect with:"
    echo "     - VNC client: localhost:5901 (password: vncpass)"
    echo "     - Browser: ./scripts/start-novnc.sh then http://localhost:6080/vnc.html"
    echo ""

    docker run "${DOCKER_ARGS[@]}" \
        --publish=6080:6080 \
        --publish=5901:5901 \
        --env="LIBGL_ALWAYS_SOFTWARE=1" \
        "$IMAGE_NAME" \
        /bin/bash
fi
