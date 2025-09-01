#!/bin/bash

# ORB-SLAM2 ROS2 Build Script
# This script helps build the ROS2 version of ORB-SLAM2

set -e

echo "Building ORB-SLAM2 ROS2..."

# Check if we're in a ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 environment not sourced. Please source your ROS2 installation first."
    echo "Example: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS2 Distribution: $ROS_DISTRO"

# Check if colcon is available
if ! command -v colcon &> /dev/null; then
    echo "Error: colcon not found. Please install colcon-common-extensions:"
    echo "sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# Check dependencies
echo "Checking dependencies..."

# Check for OpenCV
if ! pkg-config --exists opencv4; then
    if ! pkg-config --exists opencv; then
        echo "Warning: OpenCV not found via pkg-config. Make sure OpenCV is installed."
    fi
fi

# Check for Eigen3
if ! pkg-config --exists eigen3; then
    echo "Warning: Eigen3 not found via pkg-config. Make sure Eigen3 is installed."
fi

# Check for Boost
if ! pkg-config --exists boost; then
    echo "Warning: Boost not found via pkg-config. Make sure Boost is installed."
fi

# Install ROS2 dependencies if needed
echo "Installing ROS2 dependencies..."
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-std-srvs \
    ros-$ROS_DISTRO-rcl-interfaces \
    ros-$ROS_DISTRO-message-filters

# Build the package
echo "Building with colcon..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo ""
    echo "To use the package:"
    echo "1. Source the workspace: source install/setup.bash"
    echo "2. Run a node: ros2 run orb_slam2_ros orb_slam2_ros_mono"
    echo "3. Or use a launch file: ros2 launch orb_slam2_ros orb_slam2_mono.launch.py"
else
    echo "Build failed!"
    exit 1
fi
