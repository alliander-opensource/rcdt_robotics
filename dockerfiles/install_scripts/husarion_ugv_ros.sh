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
git clone -b humble https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros

export HUSARION_ROS_BUILD_TYPE=simulation

vcs import src < src/husarion_ugv_ros/husarion_ugv/${HUSARION_ROS_BUILD_TYPE}_deps.repos

rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /home/$UNAME/.bashrc

colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

echo "source /home/$UNAME/husarion_ws/install/setup.bash" >>/home/$UNAME/.bashrc