# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG COLCON_BUILD_SEQUENTIAL
ENV UNAME=rcdt
ENV ROS_DISTRO=jazzy

# Install core packages
RUN apt-get update \
  && curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash \
  && apt-get install -y --no-install-recommends \
    flake8 \
    git-lfs \
    htop \
    nano \
    python3-pip \
    zstd

# Install ROS dependencies @TODO some of these can be moved to other files
RUN apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-launch-pytest \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-servo \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# Python dependencies
RUN pip install uv --break-system-packages
RUN echo "export PYTHONPATH=\"/home/$UNAME/rcdt_robotics/.venv/lib/python3.12/site-packages:\$PYTHONPATH\"" \
  >> /home/$UNAME/.bashrc \
  && echo "export PATH=\"/home/$UNAME/rcdt_robotics/.venv/bin:\$PATH\"" \
  >> /home/$UNAME/.bashrc

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

# Install vizanti
RUN apt-get update \
  && mkdir -p /home/$UNAME/vizanti_ws/src \
  && cd /home/$UNAME/vizanti_ws/src \
  && git clone -b ros2 https://github.com/MoffKalast/vizanti.git \
  && git clone -b jazzy https://github.com/alliander-opensource/rws.git \
  && cd /home/$UNAME/vizanti_ws \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i

WORKDIR /home/$UNAME/vizanti_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --symlink-install \
  && echo "source /home/$UNAME/vizanti_ws/install/setup.bash" >>/home/$UNAME/.bashrc

ENTRYPOINT ["/bin/bash"]
