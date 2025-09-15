# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

INCLUDE ./general/pre_install.dockerfile

COPY ./install_scripts/cuda.sh .
RUN ./cuda.sh

COPY ./install_scripts/ros2_jazzy.sh .
RUN ./ros2_jazzy.sh

COPY ./install_scripts/core_packages.sh .
RUN ./core_packages.sh

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

INCLUDE ./general/post_install.dockerfile