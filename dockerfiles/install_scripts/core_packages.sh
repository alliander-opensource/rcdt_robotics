#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

apt update
apt install -y \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-xacro \
    ros-humble-rqt-tf-tree \
    htop \
    python3-pip \
    git-lfs
