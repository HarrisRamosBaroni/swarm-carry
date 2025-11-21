#!/bin/bash
# Build script: Compile ROS2 workspace
set -e

echo "=== Building ROS2 Workspace ==="

# Check if we're in workspace root
if [ ! -d "src" ]; then
    echo "Error: Must run from workspace root (directory containing 'src/')"
    exit 1
fi

# Set the default build type (can be overridden with BUILD_TYPE env var)
BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}

echo "Build type: $BUILD_TYPE"

# Build workspace
# Set SETUPTOOLS_USE_DISTUTILS to avoid editable install issues
export SETUPTOOLS_USE_DISTUTILS=stdlib

colcon build \
    --merge-install \
    --cmake-args \
    "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
    "-DBUILD_TESTING=ON" \
    -Wall -Wextra -Wpedantic

echo ""
echo "✓ Build complete!"
echo "To use the workspace, run: source install/setup.bash"
