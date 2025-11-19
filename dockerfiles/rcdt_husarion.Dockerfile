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
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# Install nav2 plugins
RUN mkdir -p /home/$UNAME/nav2_plugins_ws/src \
  && cd /home/$UNAME/nav2_plugins_ws/src \
  && git clone -b jazzy-devel https://github.com/blackcoffeerobotics/vector_pursuit_controller.git \
  && cd /home/$UNAME/nav2_plugins_ws \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i

WORKDIR /home/$UNAME/nav2_plugins_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \ 
  --event-handlers console_direct+ \
  && echo "source /home/$UNAME/nav2_plugins_ws/install/setup.bash" >>/home/$UNAME/.bashrc

RUN apt-get update \
  && mkdir -p /home/$UNAME/husarion_ws/src \
  && cd /home/$UNAME/husarion_ws
  
RUN git clone -b 2.3.1 https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros \
  && export HUSARION_ROS_BUILD_TYPE=simulation \ 
  && vcs import src < src/husarion_ugv_ros/husarion_ugv/${HUSARION_ROS_BUILD_TYPE}_deps.repos \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  && echo "source /home/$UNAME/husarion_ws/install/setup.bash" >>/home/$UNAME/.bashrc

# Install dev packages
COPY dev-pkgs.txt /home/$UNAME/dev-pkgs.txt
RUN apt-get update && apt-get install -y -qq --no-install-recommends  \
    `cat /home/$UNAME/dev-pkgs.txt`\
    && rm -rf /var/lib/apt/lists/* \
    && apt-get autoremove \
    && apt-get clean

INCLUDE rcdt_post_install.Dockerfile
