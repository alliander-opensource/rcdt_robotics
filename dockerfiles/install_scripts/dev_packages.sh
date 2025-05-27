#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox 

curl -LsSf https://astral.sh/uv/install.sh | sh

echo "source /home/$UNAME/.local/bin/env" >> /home/$UNAME/.bashrc