#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
apt install -y \
    flake8 \
    git-lfs \
    htop \
    nano \
    python3-pip

apt install -y \
    ros-$ROS_DISTRO-launch-pytest \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-servo \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-velodyne \
    ros-$ROS_DISTRO-velodyne-description

pip install uv --break-system-packages

echo "export PYTHONPATH=\"/home/$UNAME/rcdt_robotics/.venv/lib/python3.12/site-packages:\$PYTHONPATH\"" \
  >> /home/$UNAME/.bashrc

echo "export PATH=\"/home/$UNAME/rcdt_robotics/.venv/bin:\$PATH\"" \
  >> /home/$UNAME/.bashrc