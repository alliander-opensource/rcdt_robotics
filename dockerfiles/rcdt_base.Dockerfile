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

# Remove existing user with same UID if exists, then add new user:
RUN \
  if getent passwd $UID; then userdel $(id -nu $UID); else :; fi \
  && useradd -m -u $UID -p "$(openssl passwd -1 $UNAME)" $UNAME \
  && usermod -aG sudo,dialout $UNAME

# Create bashrc file:
RUN echo "if test -f ~/.personal.bashrc; then\nsource ~/.personal.bashrc\nfi" >> /home/$UNAME/.bashrc \
  && echo "if test -f ~/.env; then\nset -a && source ~/.env && set +a\nfi" >> /home/$UNAME/.bashrc

# Add ROS2 to apt sources
RUN apt update && apt install -y -qq --no-install-recommends \
  bash \
  curl \
  git \
  nano \
  nvim \
  software-properties-common \
  && add-apt-repository universe \
  && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
&& curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release \
&& echo $VERSION_CODENAME)_all.deb" \
&& dpkg -i /tmp/ros2-apt-source.deb

# Install ROS2
# two-stage install is because otherwise it can't find dependencies in ARM build
RUN apt update && apt install -y --no-install-recommends \
  ros-dev-tools \
  && apt install -y --no-install-recommends \
  ros-$ROS_DISTRO-desktop \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# Run rosdep
RUN rosdep init \
  && rosdep update

CMD ["sleep", "infinity"]
