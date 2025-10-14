#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

cd /home/$UNAME
mkdir zed_ws
cd /home/$UNAME/zed_ws
git clone -b jazzy https://github.com/stereolabs/zed-ros2-wrapper.git src/zed_ros2_wrapper

sudo apt update
rosdep update
rosdep install --from-paths src --rosdistro $ROS_DISTRO -y -r
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

echo "source /home/$UNAME/zed_ws/install/setup.bash" >>/home/$UNAME/.bashrc
