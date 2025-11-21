#!/bin/bash
# Run script: Launch the main simulation
set -e

echo "=== Launching Simulation ==="

# Check if workspace is sourced
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "Error: Workspace not sourced!"
    echo "Run: source install/setup.bash"
    exit 1
fi

# Check if GZ_VERSION is set
if [ -z "$GZ_VERSION" ]; then
    echo "Warning: GZ_VERSION not set, setting to 'harmonic'"
    export GZ_VERSION=harmonic
fi

echo "Gazebo Version: $GZ_VERSION"
echo ""
echo "Starting simulation..."
echo "Note: Close Gazebo and press Ctrl+C to stop all nodes."
echo "Wait 30 seconds after stopping before restarting."
echo ""

# Launch the main simulation
# TODO: Update this launch command for your specific simulation
ros2 launch inverted_pendulum_bringup inverted_pendulum.launch.py

echo ""
echo "Simulation stopped."
