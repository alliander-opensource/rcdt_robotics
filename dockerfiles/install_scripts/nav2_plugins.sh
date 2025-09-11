#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

mkdir -p /home/$UNAME/nav2_plugins_ws/src
cd /home/$UNAME/nav2_plugins_ws/src
git clone -b jazzy-devel https://github.com/blackcoffeerobotics/vector_pursuit_controller.git
cd /home/$UNAME/nav2_plugins_ws

rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/nav2_plugins_ws/install/setup.bash" >>/home/$UNAME/.bashrc