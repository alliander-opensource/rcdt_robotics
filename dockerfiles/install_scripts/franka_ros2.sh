#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

#Install ros2 franka: https://support.franka.de/docs/franka_ros2.html
apt update
apt install -y \
    ros-humble-angles \
    ros-humble-generate-parameter-library \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-pinocchio \
    ros-humble-realtime-tools \
    ros-humble-hardware-interface

mkdir -p /home/$UNAME/franka_ws/src
cd /home/$UNAME/franka_ws
git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2
cd /home/$UNAME/franka_ws/src/franka_ros2
git checkout v0.1.15

cd /home/$UNAME/franka_ws
source /home/$UNAME/.bashrc
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore franka_ign_ros2_control
echo "source /home/$UNAME/franka_ws/install/setup.bash" >>/home/$UNAME/.bashrc
