# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG COLCON_BUILD_SEQUENTIAL
ENV ROS_DISTRO=jazzy

# Install ROS dependencies 
RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# Install nav2 plugins
RUN mkdir -p /rcdt/nav2_plugins_ws/src \
  && cd /rcdt/nav2_plugins_ws/src \
  && git clone -b jazzy-devel https://github.com/blackcoffeerobotics/vector_pursuit_controller.git \
  && cd /rcdt/nav2_plugins_ws \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i

WORKDIR /rcdt/nav2_plugins_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \ 
  --event-handlers console_direct+ \
  && echo "source /rcdt/nav2_plugins_ws/install/setup.bash" >>/root/.bashrc

RUN apt update \
  && mkdir -p /rcdt/husarion_ws/src \
  && cd /rcdt/husarion_ws \
  && git clone -b 2.3.1 https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros \
  && export HUSARION_ROS_BUILD_TYPE=simulation \ 
  && vcs import src < src/husarion_ugv_ros/husarion_ugv/${HUSARION_ROS_BUILD_TYPE}_deps.repos \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  && echo "source /rcdt/husarion_ws/install/setup.bash" >>/root/.bashrc

# Install dev packages
COPY dev-pkgs.txt /rcdt/dev-pkgs.txt
RUN apt update && apt install -y -qq --no-install-recommends  \
    `cat /rcdt/dev-pkgs.txt`\
    && rm -rf /var/lib/apt/lists/* \
    && apt autoremove \
    && apt clean

WORKDIR /rcdt
ENTRYPOINT ["/rcdt/entrypoint.sh"]
CMD ["sleep", "infinity"]
