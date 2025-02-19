# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

pip install pyrealsense2
apt update
apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description
