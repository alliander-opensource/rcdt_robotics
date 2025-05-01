#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

# Moveit Visual Tools:
mkdir -p /home/$UNAME/moveit_visual_tools/src
cd /home/$UNAME/moveit_visual_tools/src
git clone -b ros2 https://github.com/ros-planning/moveit_visual_tools
vcs import <moveit_visual_tools/moveit_visual_tools.repos

cd /home/$UNAME/moveit_visual_tools
. /opt/ros/humble/setup.sh
. /home/$UNAME/moveit_ws/install/setup.sh
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/moveit_visual_tools/install/setup.bash" >>/home/$UNAME/.bashrc

# Bio IK:
mkdir -p /home/$UNAME/bio_ik_ws/src
cd /home/$UNAME/bio_ik_ws/src
git clone https://github.com/PickNikRobotics/bio_ik.git -b ros2

cd /home/$UNAME/bio_ik_ws/
source /home/$UNAME/.bashrc
colcon build
echo "source /home/$UNAME/bio_ik_ws/install/setup.bash" >>/home/$UNAME/.bashrc
