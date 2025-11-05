#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

cd /home/$UNAME && \
    mkdir realsense_ws && \
    cd /home/$UNAME/realsense_ws && \
    git clone -b 4.57.2 https://github.com/IntelRealSense/realsense-ros.git src/realsense_ros

apt update && apt-get install python3-rosdep -y
rosdep update && rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
colcon build

echo "source /home/$UNAME/realsense_ws/install/setup.bash" >>/home/$UNAME/.bashrc
