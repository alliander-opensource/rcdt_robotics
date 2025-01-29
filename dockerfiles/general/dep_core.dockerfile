# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Core dependencies:
RUN apt update && apt install -y ros-humble-ament-cmake-clang-format
RUN apt update && apt install -y ros-humble-ros2-controllers
RUN apt update && apt install -y ros-humble-ros2-control
RUN apt update && apt install -y ros-humble-ros2-control-test-assets
RUN apt update && apt install -y ros-humble-controller-manager
RUN apt update && apt install -y ros-humble-control-msgs
RUN apt update && apt install -y ros-humble-control-toolbox
RUN apt update && apt install -y ros-humble-xacro
RUN apt update && apt install -y ros-humble-rqt-tf-tree
RUN apt update && apt install -y htop
RUN apt update && apt install -y python3-pip
RUN apt update && apt install -y git-lfs