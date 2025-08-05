#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
apt update

UBUNTU_RELEASE_YEAR=22    
CUDA_MAJOR=11             
ZED_SDK_MAJOR=5
ZED_SDK_MINOR=0

apt-get update -y
apt-get install --no-install-recommends -y wget zstd udev libgomp1

wget -q --content-disposition --trust-server-names \
     "https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR}"


installer=$(ls ZED_SDK*run)

chmod +x "$installer"

"./$installer" silent skip_tools skip_cuda

rm "$installer"

cd /home/$UNAME
mkdir zed_ws
cd /home/$UNAME/zed_ws
git clone https://github.com/stereolabs/zed-ros2-wrapper.git src/zed_ros2_wrapper

rosdep install --from-paths src --ignore-src -r -y # install dependencies
source /home/$UNAME/.bashrc

colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/zed_ws/install/setup.bash" >>/home/$UNAME/.bashrc

ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so