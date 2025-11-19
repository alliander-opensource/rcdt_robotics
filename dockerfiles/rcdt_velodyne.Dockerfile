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
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  libpcap-dev \
  ros-$ROS_DISTRO-velodyne-description \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

RUN mkdir -p /home/$UNAME/velodyne_ws/src \
  && cd /home/$UNAME/velodyne_ws \
  && git clone -b ros2 https://github.com/alliander-opensource/velodyne.git \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && echo "source /home/$UNAME/velodyne_ws/install/setup.bash" >>/home/$UNAME/.bashrc

# Install dev packages
COPY dev-pkgs.txt /home/$UNAME/dev-pkgs.txt
RUN apt-get update && apt-get install -y -qq --no-install-recommends  \
    `cat /home/$UNAME/dev-pkgs.txt`\
    && rm -rf /var/lib/apt/lists/* \
    && apt-get autoremove \
    && apt-get clean

INCLUDE rcdt_post_install.Dockerfile
