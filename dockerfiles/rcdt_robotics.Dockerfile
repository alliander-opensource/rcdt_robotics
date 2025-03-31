# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

INCLUDE ./general/pre_install.dockerfile

COPY ./install_scripts/core_packages.sh .
RUN ./core_packages.sh

COPY ./install_scripts/gazebo.sh .
RUN ./gazebo.sh

COPY ./install_scripts/moveit.sh .
RUN ./moveit.sh

COPY ./install_scripts/moveit_tools.sh .
RUN ./moveit_tools.sh

COPY ./install_scripts/libfranka.sh .
RUN ./libfranka.sh

COPY ./install_scripts/franka_ros2.sh .
RUN ./franka_ros2.sh

COPY ./install_scripts/panther_ros.sh .
RUN ./panther_ros.sh

COPY ./install_scripts/rosboard.sh .
RUN ./rosboard.sh

COPY ./install_scripts/pyflow.sh .
RUN ./pyflow.sh

COPY ./install_scripts/sensors.sh .
RUN ./sensors.sh

COPY ./install_scripts/calibration.sh .
RUN ./calibration.sh

COPY ./install_scripts/sphinx.sh .
RUN ./sphinx.sh

COPY ./install_scripts/dev_packages.sh .
RUN ./dev_packages.sh

INCLUDE ./general/post_install.dockerfile