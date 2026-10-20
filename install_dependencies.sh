#!/bin/bash
# Install the apt packages needed by ur5_moveit_toolkit (Ubuntu 24.04, ROS 2 Jazzy).
# Usage: ./install_dependencies.sh            (asks for sudo)
set -e

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ROS 2 Jazzy not found in /opt/ros/jazzy."
    echo "Install it first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
    exit 1
fi

sudo apt update
sudo apt install -y \
    ros-jazzy-ur \
    ros-jazzy-ur-robot-driver \
    ros-jazzy-ur-moveit-config \
    ros-jazzy-ur-calibration \
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-pymoveit2 \
    ros-jazzy-realsense2-camera \
    ros-jazzy-rqt-image-view \
    python3-colcon-common-extensions \
    python3-numpy python3-scipy python3-serial

# Serial port access for the gripper (/dev/ttyUSB0). Takes effect after logging out and in.
sudo usermod -aG dialout "$USER"

echo
echo "Done. Now build the workspace:"
echo "  cd ~/ur5_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
