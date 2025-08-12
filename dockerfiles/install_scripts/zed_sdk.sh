#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Based on dockerfile of ZED-ros2-wrapper at: 
# https://github.com/stereolabs/zed-ros2-wrapper/blob/e9f54907fbf41ee9ce5d54f3bb694af93dad8bb3/docker/Dockerfile.desktop-humble

set -e

UBUNTU_RELEASE_YEAR=24      
CUDA_MAJOR=12              
CUDA_MINOR=9                
ZED_SDK_MAJOR=5
ZED_SDK_MINOR=0             

###  CUDA “version.txt” marker 
echo "CUDA Version ${CUDA_MAJOR}.${CUDA_MINOR}.0" > /usr/local/cuda/version.txt || true

###  System prerequisites
apt-get update -y \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       lsb-release wget less udev sudo zstd build-essential cmake \
       python3 python3-pip libpng-dev libgomp1

###  Download + install ZED SDK 
installer="ZED_SDK_Ubuntu${UBUNTU_RELEASE_YEAR}_cuda${CUDA_MAJOR}.${CUDA_MINOR}.run"
sdk_url="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR}"

echo "Downloading ${sdk_url}  →  ${installer} ..."
wget -q -O "${installer}" "${sdk_url}"
chmod +x "${installer}"

echo "Running installer …"
./"${installer}" -- silent 

###  Post-install

ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0  /usr/lib/x86_64-linux-gnu/libusb-1.0.so

chmod 755 /usr/local/zed/lib
find /usr/local/zed/lib -type f -name 'libsl_*.so*' -exec chmod 644 {} +
chmod -R u+rwX /usr/local/zed/resources /usr/local/zed/settings
chmod o+rx /usr/local/zed /usr/local/zed/lib
chmod a+r /usr/local/zed/lib/libsl_zed.so

rm -f "${installer}"
rm -rf /var/lib/apt/lists/*