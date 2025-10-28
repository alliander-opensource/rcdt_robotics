# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG COLCON_BUILD_SEQUENTIAL
ENV UNAME=rcdt
ENV UID=1000
ENV ROS_DISTRO=jazzy

COPY ./install_scripts/core_packages.sh .
RUN ./core_packages.sh

COPY ./install_scripts/zed_sdk.sh .
RUN ./zed_sdk.sh

COPY ./install_scripts/zed_wrapper.sh .
RUN ./zed_wrapper.sh

COPY ./install_scripts/franka_ros2.sh .
RUN ./franka_ros2.sh

COPY ./install_scripts/husarion_ugv_ros.sh .
RUN ./husarion_ugv_ros.sh

COPY ./install_scripts/nav2_plugins.sh .
RUN ./nav2_plugins.sh

COPY ./install_scripts/velodyne_ros.sh .
RUN ./velodyne_ros.sh

COPY ./install_scripts/vizanti.sh .
RUN ./vizanti.sh

COPY ./install_scripts/franka_lock_unlock.sh .
RUN ./franka_lock_unlock.sh

COPY ./install_scripts/dev_packages.sh .
RUN ./dev_packages.sh

RUN chown -R $UNAME /home/$UNAME
USER $UNAME
WORKDIR /home/$UNAME

#Set entrypoint to bash:
ENTRYPOINT ["/bin/bash"]
