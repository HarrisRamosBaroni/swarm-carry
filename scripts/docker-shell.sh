#!/bin/bash
# Open an interactive bash shell in the running container

set -e

CONTAINER_NAME="swarm-sim-dev"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running"
    echo ""
    echo "Start it first with one of:"
    echo "  ./scripts/docker-run.sh"
    echo "  VSCode: Reopen in Container"
    exit 1
fi

echo "Opening shell in container '${CONTAINER_NAME}'..."
echo ""

# Open interactive shell as ros user
docker exec -it -u ros "${CONTAINER_NAME}" \
    /bin/bash -c "cd /workspace && source install/setup.bash 2>/dev/null || true && bash"
