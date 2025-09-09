#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

apt install -y \
    libpcap-dev \
    ros-$ROS_DISTRO-velodyne-description

mkdir -p /home/$UNAME/velodyne_ws/src
cd /home/$UNAME/velodyne_ws
git clone -b ros2 https://github.com/alliander-opensource/velodyne.git # In our fork we use the Sensor Data QOS profile.

rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/velodyne_ws/install/setup.bash" >>/home/$UNAME/.bashrc