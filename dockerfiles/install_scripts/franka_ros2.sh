#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

#Install ros2 franka: https://github.com/frankaemika/franka_ros2/tree/v1.0.2
apt update
source /home/$UNAME/.bashrc
mkdir -p /home/$UNAME/franka_ws/src
cd /home/$UNAME/franka_ws

# Clone a fork with jazzy support, since Franka offers no support yet:
git clone -b jazzy https://github.com/lianghongzhuo/franka_ros2.git src/franka_ros2

# Remove the dependencies in the package.xml that are not ready for jazzy but we don't use:
sed -i "/franka_ign_ros2_control/d" /home/$UNAME/franka_ws/src/franka_ros2/franka_ros2/package.xml
sed -i "/franka_gazebo_bringup/d" /home/$UNAME/franka_ws/src/franka_ros2/franka_ros2/package.xml
sed -i "/franka_example_controllers/d" /home/$UNAME/franka_ws/src/franka_ros2/franka_ros2/package.xml

# Only install: franka_gripper, franka_hardware, franka_msgs, libfranka, franka_description
vcs import src --recursive --skip-existing <src/franka_ros2/franka.repos
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select franka_gripper franka_hardware franka_msgs libfranka franka_description
echo "source /home/$UNAME/franka_ws/install/setup.bash" >>/home/$UNAME/.bashrc
