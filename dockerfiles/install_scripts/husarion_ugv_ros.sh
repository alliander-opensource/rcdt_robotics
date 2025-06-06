#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
apt update

#Install husarion_ugv_ros: https://github.com/husarion/husarion_ugv_ros
cd /home/$UNAME
mkdir husarion_ws
cd /home/$UNAME/husarion_ws
git clone -b 2.1.2 https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros
export HUSARION_ROS_BUILD_TYPE=simulation
vcs import src < src/husarion_ugv_ros/panther/panther_$HUSARION_ROS_BUILD_TYPE.repos

cp -r src/ros2_controllers/diff_drive_controller src
cp -r src/ros2_controllers/imu_sensor_broadcaster src
rm -rf src/ros2_controllers

rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /home/$UNAME/.bashrc
colcon build --packages-up-to panther --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/husarion_ws/install/setup.bash" >>/home/$UNAME/.bashrc