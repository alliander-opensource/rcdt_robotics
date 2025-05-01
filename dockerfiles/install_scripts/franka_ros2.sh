#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

#Install ros2 franka: https://github.com/frankaemika/franka_ros2/tree/v1.0.0
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
cd /home/$UNAME/franka_ws/src
git clone -b v1.0.0 https://github.com/frankaemika/franka_ros2.git
git clone -b 0.3.0 https://github.com/frankaemika/franka_description.git
cd /home/$UNAME/franka_ws/
rosdep install --from-paths src --ignore-src --rosdistro humble -y
source /home/$UNAME/.bashrc
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore franka_ign_ros2_control
echo "source /home/$UNAME/franka_ws/install/setup.bash" >>/home/$UNAME/.bashrc
