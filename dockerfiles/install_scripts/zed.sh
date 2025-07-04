#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# This script installs the ZED SDK 5.0.3 on a Docker image based on Ubuntu 22.04 with CUDA 12.8.

UBUNTU_RELEASE_YEAR=22    
CUDA_MAJOR=12             
ZED_SDK_MAJOR=5
ZED_SDK_MINOR=0

sudo apt-get update -y
sudo apt-get install --no-install-recommends -y wget zstd udev libgomp1

wget --continue --content-disposition --trust-server-names \
     "https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR}"