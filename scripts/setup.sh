#!/bin/bash
# Setup script: Install dependencies for ROS2 workspace
set -e

echo "=== ROS2 Workspace Setup ==="

# Check if ROS_DISTRO is set
if [ -z "$ROS_DISTRO" ]; then
    echo "Warning: ROS_DISTRO not set, defaulting to 'jazzy'"
    export ROS_DISTRO=jazzy
fi

# Set Gazebo version
if [ -z "$GZ_VERSION" ]; then
    echo "Setting GZ_VERSION=harmonic"
    export GZ_VERSION=harmonic
fi

echo "ROS Distribution: $ROS_DISTRO"
echo "Gazebo Version: $GZ_VERSION"

# Update package lists
echo "Updating apt package lists..."
sudo apt-get update

# Update rosdep
echo "Updating rosdep..."
rosdep update --rosdistro=$ROS_DISTRO

# Install dependencies from package.xml files
echo "Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro=$ROS_DISTRO

echo ""
echo "✓ Setup complete!"
echo "Next steps:"
echo "  1. Export GZ_VERSION: export GZ_VERSION=harmonic"
echo "  2. Build workspace: ./scripts/build.sh"
echo "  3. Source workspace: source install/setup.bash"
