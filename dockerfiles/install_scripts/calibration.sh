# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
apt update

# gazebo_apriltag:
cd
git clone --recurse-submodules https://github.com/rickarmstrong/gazebo_apriltag.git
python3 gazebo_apriltag/generate.py

# apriltag_ros:
apt install -y ros-humble-apriltag-ros

# easy_handeye2:
mkdir -p /home/$UNAME/calibration_ws/src
cd /home/$UNAME/calibration_ws
git clone https://github.com/marcoesposito1988/easy_handeye2.git src/easy_handeye2
. /opt/ros/humble/setup.sh
colcon build
echo "source /home/$UNAME/calibration_ws/install/setup.bash" >>/home/$UNAME/.bashrc