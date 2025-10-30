# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG COLCON_BUILD_SEQUENTIAL
ENV UNAME=rcdt
ENV ROS_DISTRO=jazzy

# Franka ROS2 library
RUN apt-get update \
  && mkdir -p /home/$UNAME/franka_ws/src \
  && cd /home/$UNAME/franka_ws

RUN git clone -b v3.0.0 https://github.com/frankarobotics/franka_ros2.git src/franka_ros2 \
  && vcs import src --recursive --skip-existing <src/franka_ros2/franka.repos \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i 

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && echo "source /home/$UNAME/franka_ws/install/setup.bash" >>/home/$UNAME/.bashrc

# Franka lock_unlock
RUN . /home/$UNAME/.bashrc \
  && apt-get update \
  && mkdir -p /home/$UNAME/franka_lock_unlock/src \
  && cd /home/$UNAME/franka_lock_unlock

RUN git clone https://github.com/alliander-opensource/franka_lock_unlock.git \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i 

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && echo "source /home/$UNAME/franka_lock_unlock/install/setup.bash" >>/home/$UNAME/.bashrc

# Install dev packages
COPY dev-pkgs.txt /home/$UNAME/dev-pkgs.txt
RUN apt-get update && apt-get install -y -qq --no-install-recommends  \
    `cat /home/$UNAME/dev-pkgs.txt`\
    && rm -rf /var/lib/apt/lists/* \
    && apt-get autoremove \
    && apt-get clean

ENTRYPOINT ["/bin/bash"]
