#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

# Packages without specific version:
pip install pytest-timeout
pip install ruff
pip install ty
pip install termcolor
pip install pydoclint[flake8]
pip install Flake8-pyproject

# Specifying numpy range to work with ROS and Ultralytics:
pip install "numpy>=1.23.0,<2.0"

# Specifying currently newest version of transforms3d to avoid conflict with imported numpy.float in older version.
pip install "transforms3d>=0.4.2"

sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox 