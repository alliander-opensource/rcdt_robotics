# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG COLCON_BUILD_SEQUENTIAL
ENV UNAME=rcdt
ENV ROS_DISTRO=jazzy

# Install ROS dependencies 
RUN apt-get install -y --no-install-recommends \
  ros-$ROS_DISTRO-ros-gz \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremo

# Install dev packages
COPY dev-pkgs.txt /home/$UNAME/dev-pkgs.txt
RUN apt-get update && apt-get install -y -qq --no-install-recommends  \
    `cat /home/$UNAME/dev-pkgs.txt`\
    && rm -rf /var/lib/apt/lists/* \
    && apt-get autoremove \
    && apt-get clean

INCLUDE rcdt_post_install.Dockerfile
