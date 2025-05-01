#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


set -e
apt update

# Realsense:
apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description
pip install pyrealsense2

# Velodyne:
apt install -y ros-humble-velodyne-description