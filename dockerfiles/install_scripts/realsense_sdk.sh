#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

cd /home/$UNAME
mkdir realsense_ws
cd /home/$UNAME/realsense_ws
git clone -b 4.57.3 git@github.com:IntelRealSense/realsense-ros.git

sudo apt update
rosdep update
rosdep install --from-paths src --rosdistro $ROS_DISTRO -y -r
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

echo "source /home/$UNAME/realsense_ws/install/setup.bash" >>/home/$UNAME/.bashrc
