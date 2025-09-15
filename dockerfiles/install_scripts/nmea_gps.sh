#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

mkdir -p /home/$UNAME/nmea_gps_ws/src
cd /home/$UNAME/nmea_gps_ws

git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git src/nmea_navsat_driver

rosdep update --rosdistro "$ROS_DISTRO"
rosdep install -i --from-path src --rosdistro "$ROS_DISTRO" -y

source "/opt/ros/$ROS_DISTRO/setup.bash"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "source /home/$UNAME/nmea_gps_ws/install/setup.bash" >>/home/$UNAME/.bashrc