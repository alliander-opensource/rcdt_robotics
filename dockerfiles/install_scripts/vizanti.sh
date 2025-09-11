#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

mkdir -p /home/$UNAME/vizanti_ws/src
cd /home/$UNAME/vizanti_ws/src
git clone -b ros2 https://github.com/MoffKalast/vizanti.git
git clone -b jazzy https://github.com/alliander-opensource/rws.git

cd /home/$UNAME/vizanti_ws
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

colcon build --symlink-install
echo "source /home/$UNAME/vizanti_ws/install/setup.bash" >>/home/$UNAME/.bashrc