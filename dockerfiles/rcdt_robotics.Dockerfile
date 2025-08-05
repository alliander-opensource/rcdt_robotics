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

COPY ./install_scripts/franka_ros2.sh .
RUN ./franka_ros2.sh
ENV GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:/home/$UNAME/franka_ws/src"

COPY ./install_scripts/husarion_ugv_ros.sh .
RUN ./husarion_ugv_ros.sh

COPY ./install_scripts/rosboard.sh .
RUN ./rosboard.sh

COPY ./install_scripts/pyflow.sh .
RUN ./pyflow.sh

COPY ./install_scripts/sensors.sh .
RUN ./sensors.sh

COPY ./install_scripts/sphinx.sh .
RUN ./sphinx.sh

COPY ./install_scripts/franka_lock_unlock.sh .
RUN ./franka_lock_unlock.sh

COPY ./install_scripts/dev_packages.sh .
RUN ./dev_packages.sh

COPY ./install_scripts/zed.sh .
RUN ./zed.sh

INCLUDE ./general/post_install.dockerfile