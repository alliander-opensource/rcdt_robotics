#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

mkdir -p /home/$UNAME/dev_packages/src
cd /home/$UNAME/dev_packages/src
git clone -b v2.0.12-jazzy https://github.com/Box-Robotics/ros2_numpy.git

cd /home/$UNAME/dev_packages

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "source /home/$UNAME/dev_packages/install/setup.bash" >>/home/$UNAME/.bashrc
