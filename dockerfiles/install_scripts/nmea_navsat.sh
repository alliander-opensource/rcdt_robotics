#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

UNAME=rcdt
ROS_DISTRO=jazzy
set -e
source /home/$UNAME/.bashrc
apt update

mkdir -p /home/$UNAME/nmea_navsat_ws/src
cd /home/$UNAME/nmea_navsat_ws

git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git src/nmea_navsat_driver

rosdep update --rosdistro "$ROS_DISTRO"
rosdep install -i --from-path src --rosdistro "$ROS_DISTRO" -y

colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

echo "source /home/$UNAME/nmea_navsat_ws/install/setup.bash" >>/home/$UNAME/.bashrc