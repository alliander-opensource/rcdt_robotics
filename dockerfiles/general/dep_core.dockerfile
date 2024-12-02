# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Core dependencies:
RUN apt install -y ros-humble-ament-cmake-clang-format
RUN apt install -y ros-humble-ros2-controllers
RUN apt install -y ros-humble-ros2-control
RUN apt install -y ros-humble-ros2-control-test-assets
RUN apt install -y ros-humble-controller-manager
RUN apt install -y ros-humble-control-msgs
RUN apt install -y ros-humble-control-toolbox
RUN apt install -y ros-humble-xacro
RUN apt install -y python3-pip