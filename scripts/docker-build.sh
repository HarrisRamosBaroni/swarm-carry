#!/bin/bash
# Build the Docker image without running it
# Useful for rebuilding after Dockerfile changes

set -e

IMAGE_NAME="swarm-sim"

echo "Building Docker image '$IMAGE_NAME'..."
echo "(This includes VNC server - takes ~5 minutes first time)"
echo ""

docker build -t "$IMAGE_NAME" .devcontainer/

echo ""
echo "✓ Image '$IMAGE_NAME' built successfully!"
echo ""
echo "To run the container:"
echo "  ./scripts/docker-run.sh"
