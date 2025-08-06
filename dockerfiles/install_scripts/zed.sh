#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
apt update

UBUNTU_RELEASE_YEAR=22    
CUDA_MAJOR=11
CUDA_MINOR=8            
ZED_SDK_MAJOR=5
ZED_SDK_MINOR=0
ZED_SDK_VERSION=5.0.3 

sudo apt-get update -y
sudo apt-get install --no-install-recommends -y zstd

echo "Downloading ZED SDK for Ubuntu ${UBUNTU_RELEASE_YEAR} with CUDA ${CUDA_MAJOR}..."

installer="ZED_SDK_Ubuntu${UBUNTU_RELEASE_YEAR}_cuda${CUDA_MAJOR}.${CUDA_MINOR}_v${ZED_SDK_VERSION}.zstd.run"
url="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR}"

echo "Downloading ${url}"

wget -qO "${installer}" "${url}"

chmod +x "${installer}"

echo "Installing ZED SDK from $installer"

sudo "./$installer" silent skip_tools skip_cuda

rm "$installer"

cd /home/$UNAME
mkdir zed_ws
cd /home/$UNAME/zed_ws
git clone https://github.com/stereolabs/zed-ros2-wrapper.git src/zed_ros2_wrapper

rosdep install --from-paths src --rosdistro $ROS_DISTRO -y -r
source /home/$UNAME/.bashrc

colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/zed_ws/install/setup.bash" >>/home/$UNAME/.bashrc

ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so
