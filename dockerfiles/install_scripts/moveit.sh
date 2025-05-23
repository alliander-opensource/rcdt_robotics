#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
set -e

mkdir -p /home/$UNAME/moveit_ws/src
cd /home/$UNAME/moveit_ws/src

git clone https://github.com/moveit/moveit2.git -b main
for repo in moveit2/moveit2.repos $(
    f="moveit2/moveit2_$ROS_DISTRO.repos"
    test -r $f && echo $f
); do vcs import <"$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd /home/$UNAME/moveit_ws
source /home/$UNAME/.bashrc
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release ${COLCON_BUILD_SEQUENTIAL:+--executor sequential}
echo "source /home/$UNAME/moveit_ws/install/setup.bash" >>/home/$UNAME/.bashrc
