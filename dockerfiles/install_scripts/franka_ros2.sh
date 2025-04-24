# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

#Install ros2 franka: https://github.com/frankaemika/franka_ros2/tree/v1.0.0
apt update
apt install -y \
    ros-humble-angles \
    ros-humble-generate-parameter-library \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-pinocchio \
    ros-humble-realtime-tools \
    ros-humble-hardware-interface

mkdir -p /home/$UNAME/franka_ws/src
cd /home/$UNAME/franka_ws
git clone -b v1.0.0 https://github.com/frankaemika/franka_ros2.git src
rosdep install --from-paths src --ignore-src --rosdistro humble -y
. /opt/ros/humble/setup.sh
. /home/$UNAME/moveit_ws/install/setup.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/franka_ws/install/setup.bash" >>/home/$UNAME/.bashrc
