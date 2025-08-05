#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
export ROS_DISTRO=humble

sudo apt update
sudo apt install ros-humble-zed-msgs

cd /home/rcdt

# Create your ROS 2 Workspace if you do not have one
mkdir -p zed_ws/src/
# Move to the `src` folder of the ROS 2 Workspace
cd zed_ws/src/ 
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
# Install the required dependencies
rosdep install --from-paths src --ignore-src -r -y
# Build the wrapper
colcon build --symlink-install \
  --packages-skip zed_components
  
# Setup the environment variables
source /home/rcdt/.bashrc
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
echo "source /home/$UNAME/husarion_ws/install/setup.bash" >>/home/$UNAME/.bashrc