# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

ARG ROS_DISTRIBUTION=jazzy

#################################
#   Librealsense Builder Stage  #
#################################
FROM ros:${ROS_DISTRIBUTION}-ros-base AS librealsense-builder

ARG LIBRS_VERSION=2.57.3
# Make sure that we have a version number of librealsense as argument
RUN test -n "$LIBRS_VERSION"

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Temporarily added to avoid keyring error -------------------------
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
# END Temporarily added to avoid keyring error ---------------------

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    ca-certificates \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBRS_VERSION -o librealsense.tar.gz 
RUN tar -zxf librealsense.tar.gz \
    && rm librealsense.tar.gz
RUN ln -s /usr/src/librealsense-$LIBRS_VERSION /usr/src/librealsense

# Cmds available: rs-convert, rs-embed, rs-enumerate-devices, rs-fw-logger, rs-fw-update, rs-record, rs-terminal
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake ../ \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_CV_EXAMPLES=OFF \
    -DBUILD_WITH_TM2=OFF \
    -DBUILD_GLSL_EXTENSIONS=OFF \
    -DIMPORT_DEPTH_CAM_FW=OFF \
    -DBUILD_TOOLS=ON \
    -DCMAKE_BUILD_TYPE=Release \
    && make -j$(($(nproc)-1)) all \
    && make install 

FROM ros:${ROS_DISTRIBUTION}-ros-base

# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/

ARG USERNAME=cropteq
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DISTRIBUTION=jazzy

ENV SHELL=/bin/bash
ENV RUNNING_IN_DOCKER=true

# Set timezone
ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Make sure fzf scripts for Bash/ZSH are part of the container
RUN echo "\
# ... except FZF files ...\
path-include=/usr/share/doc/fzf/*" >> /etc/dpkg/dpkg.cfg.d/excludes


# Temporarily added to avoid keyring error -------------------------
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
# END Temporarily added to avoid keyring error ---------------------


# Update and upgrade
# Install base packages
RUN apt-get update && apt-get upgrade -y \
    &&  apt-get install -y \
        locales \
        nodejs \
        gcovr \
        python3-pip \
        python3-pytest-cov \
        autoconf \
        libtool \
        libboost-all-dev \
    && echo "Done"

# Setup locale
RUN locale-gen en_US en_US.UTF-8 C.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_TIME=C.UTF-8

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash

# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Added packages rosdep would install, so rosdep update and rosdep install can be skipped.
RUN apt-get install -y \
    ros-${ROS_DISTRIBUTION}-ament-cmake-clang-format \
    ros-${ROS_DISTRIBUTION}-ament-cmake-clang-tidy \
    ros-${ROS_DISTRIBUTION}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRIBUTION}-cv-bridge \
    libopencv-dev

# Add packages required for librealsense
RUN apt-get install -y --no-install-recommends \	
    libusb-1.0-0 \
    udev

CMD ["/bin/bash"]

# docker build -t cropteq/ros2_[ROS_DISTRIBUTION]_base:latest