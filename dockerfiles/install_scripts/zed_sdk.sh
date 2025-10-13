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

###  Download + install ZED SDK 
installer="ZED_SDK_Ubuntu${UBUNTU_RELEASE_YEAR}_cuda${CUDA_MAJOR}.${CUDA_MINOR}.run"
sdk_url="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR}"

echo "Downloading ${sdk_url}  →  ${installer} ..."
wget -q -O "${installer}" "${sdk_url}"
chmod +x "${installer}"

echo "Running installer …"
./"${installer}" -- silent 

chmod -R u+rwX,go+rX /usr/local/zed

rm -f "${installer}"
rm -rf /var/lib/apt/lists/*