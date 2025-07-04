#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# This script installs the ZED SDK 5.0.3 on a Docker image based on Ubuntu 22.04 with CUDA 12.8.

set -e
apt update

ROS_DISTRO=humble
UNAME=rcdt
source /opt/ros/$ROS_DISTRO/setup.bash

#Install husarion_ugv_ros: https://github.com/husarion/husarion_ugv_ros
cd /home/$UNAME
mkdir zed_ws
cd /home/$UNAME/zed_ws
git clone https://github.com/stereolabs/zed-ros2-wrapper.git src/zed_ros2_wrapper

rosdep install --from-paths src --ignore-src -r -y # install dependencies
source /home/$UNAME/.bashrc

colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/zed_ws/install/setup.bash" >>/home/$UNAME/.bashrc

ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so
