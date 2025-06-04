#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

#Install ros2 franka: https://github.com/frankaemika/franka_ros2/tree/v1.0.2
apt update
source /home/$UNAME/.bashrc
mkdir -p /home/$UNAME/franka_ws/src
cd /home/$UNAME/franka_ws
git clone -b v1.0.2 https://github.com/frankaemika/franka_ros2.git src
vcs import src < src/franka.repos --recursive --skip-existing
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore franka_ign_ros2_control
echo "source /home/$UNAME/franka_ws/install/setup.bash" >>/home/$UNAME/.bashrc
