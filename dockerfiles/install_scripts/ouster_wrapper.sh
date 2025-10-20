#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

cd /home/$UNAME
mkdir ouster_ws
cd /home/$UNAME/ouster_ws
git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

sudo apt update
rosdep update
rosdep install --from-paths src --rosdistro $ROS_DISTRO -y -r
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

echo "source /home/$UNAME/ouster_ws/install/setup.bash" >>/home/$UNAME/.bashrc
