#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

# Install cuda: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#debian
apt update
apt install -y wget
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
dpkg -i cuda-keyring_1.1-1_all.deb
apt update
apt install -y cuda-toolkit

# Clone our graspnet fork:
cd /home/$UNAME
git clone https://github.com/alliander-opensource/graspnet-baseline.git
