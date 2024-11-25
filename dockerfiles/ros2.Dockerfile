# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Perfrom pre-install steps:
INCLUDE ./general/pre_install.Dockerfile

# Gazebo:
RUN apt install -y ros-${ROS_DISTRO}-ros-gz

# Moveit:
RUN apt install -y ros-${ROS_DISTRO}-moveit

# Course:
RUN apt install -y ros-${ROS_DISTRO}-urdf-tutorial

#Perfrom post-install steps:
INCLUDE ./general/post_install.Dockerfile