#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/rcdt/.bashrc
apt update

apt install ros-jazzy-pointcloud-to-laserscan

sudo apt-get install -y libpcap-dev

mkdir -p /home/rcdt/velodyne_ws/src
cd /home/rcdt/velodyne_ws
git clone -b ros2 https://github.com/alliander-opensource/velodyne.git
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/rcdt/velodyne_ws/install/setup.bash" >>/home/rcdt/.bashrc

source /home/rcdt/.bashrc
