#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

sudo apt update
sudo apt install -y \
    flake8 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox 

pip install uv 

echo "export PYTHONPATH=\"/home/rcdt/rcdt_robotics/.venv/lib/python3.12/site-packages:$PYTHONPATH"" \
  >> /home/$UNAME/.bashrc

echo "export PATH=\"/home/$UNAME/rcdt_robotics/.venv/bin:$PATH"" \
  >> /home/$UNAME/.bashrc

