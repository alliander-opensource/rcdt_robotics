#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

# Instead of using https://gazebosim.org/docs/fortress/ros_installation/,
# we install ignition separately first, to obtain a newer version. Originally
# to have this issue fixed: https://github.com/gazebosim/gz-sensors/issues/454.

# Install gazebo: https://gazebosim.org/docs/fortress/install_ubuntu/
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
apt update
apt install -y ignition-fortress

# Install ros-humble-ros-gz (which will now use the already installed ignition):
apt install -y ros-humble-ros-gz

# Install ign-ros2-control from source because of https://github.com/ros-controls/gz_ros2_control/issues/503
mkdir -p /home/$UNAME/control_ws/src
cd /home/$UNAME/control_ws/src
git clone https://github.com/ros-controls/gz_ros2_control.git -b humble
cd gz_ros2_control
git checkout 0.7.15
cd /home/$UNAME/control_ws
source /home/$UNAME/.bashrc
colcon build
echo "source /home/$UNAME/control_ws/install/setup.bash" >>/home/$UNAME/.bashrc

