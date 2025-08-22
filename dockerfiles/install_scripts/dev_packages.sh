#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

mkdir -p /home/$UNAME/controller_ws/src
cd /home/$UNAME/controller_ws/src
git clone -b jazzy-devel https://github.com/blackcoffeerobotics/vector_pursuit_controller.git
cd /home/$UNAME/controller_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

apt install ros-$ROS_DISTRO-pointcloud-to-laserscan

echo "source /home/$UNAME/controller_ws/install/setup.bash" >>/home/$UNAME/.bashrc