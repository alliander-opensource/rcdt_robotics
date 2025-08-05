#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

#Install ros2 franka: https://github.com/frankaemika/franka_ros2
apt update
source /home/$UNAME/.bashrc
mkdir -p /home/$UNAME/franka_ws/src
cd /home/$UNAME/franka_ws

# Clone a fork with jazzy support, since Franka offers no support yet:
git clone -b jazzy https://github.com/WHongLin/franka_ros2.git src/franka_ros2
vcs import src --recursive --skip-existing <src/franka_ros2/franka.repos
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/franka_ws/install/setup.bash" >>/home/$UNAME/.bashrc
