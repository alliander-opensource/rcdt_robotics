# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

INCLUDE ./general/pre_install.dockerfile

COPY ./install_scripts/core_packages.sh .
RUN ./core_packages.sh

COPY ./install_scripts/franka_ros2.sh .
RUN ./franka_ros2.sh

COPY ./install_scripts/husarion_ugv_ros.sh .
RUN ./husarion_ugv_ros.sh

INCLUDE ./general/post_install.dockerfile