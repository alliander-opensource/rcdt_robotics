#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

# The GPG key expired on June 1 2025.
# A PR for the official Docker image is not finished yet: https://github.com/docker-library/official-images/pull/19162
# Therefore we install the GPG key manually.
# Also see: https://github.com/osrf/docker_images/issues/697#issuecomment-2929476986

rm /etc/apt/sources.list.d/ros2-latest.list
rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg
apt update
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
apt update
apt install /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb