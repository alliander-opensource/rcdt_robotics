#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

#Install panther_ros: https://github.com/husarion/panther_ros
cd /home/$UNAME
mkdir husarion_ws
cd /home/$UNAME/husarion_ws
git clone -b ros2 https://github.com/husarion/panther_ros.git src/panther_ros
export HUSARION_ROS_BUILD_TYPE=simulation
vcs import src <src/panther_ros/panther/panther_$HUSARION_ROS_BUILD_TYPE.repos

cp -r src/ros2_controllers/diff_drive_controller src
cp -r src/ros2_controllers/imu_sensor_broadcaster src
rm -rf src/ros2_controllers

rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /home/$UNAME/.bashrc
colcon build --packages-up-to panther --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/husarion_ws/install/setup.bash" >>/home/$UNAME/.bashrc
