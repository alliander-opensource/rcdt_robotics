#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
apt update

# Realsense:
apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description

# Velodyne:
apt install -y ros-humble-velodyne \
    ros-humble-velodyne-description

