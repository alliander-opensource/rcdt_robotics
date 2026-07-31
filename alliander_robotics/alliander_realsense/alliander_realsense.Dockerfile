# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE AS builder

##############################
# Build stage
##############################

ARG SRC_DIRECTORY
ARG COLCON_BUILD_SEQUENTIAL
ENV ROS_DISTRO=jazzy

# Install Realsense SDK:
RUN apt update && apt install -y --no-install-recommends \
  libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
  git wget cmake build-essential v4l-utils rsync unzip \
  libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

ARG TEMP_DIR="/tmp/realsense_install"
ARG ZIP_FILE="$TEMP_DIR/librealsense2.zip"
RUN mkdir -p "$TEMP_DIR"
RUN if [ $(dpkg --print-architecture) = "amd64" ]; \
  then wget -O "$ZIP_FILE" "https://github.com/realsenseai/librealsense/releases/download/v2.57.6/librealsense2_noble_x86_debians_2_57_6_beta.zip"; \
  elif [ $(dpkg --print-architecture) = "arm64" ]; \ 
  then wget -O "$ZIP_FILE" "https://github.com/realsenseai/librealsense/releases/download/v2.57.6/librealsense2_noble_ARM_debians_2_57_6_beta.zip"; \
  else echo "Unsupported architecture: $(dpkg --print-architecture)"; exit 1; fi
RUN unzip "$ZIP_FILE" -d "$TEMP_DIR" && dpkg -i "$TEMP_DIR"/*.deb && rm -rf "$TEMP_DIR"

# Install Realsense Wrapper:
WORKDIR /$WORKDIR/external
RUN apt update \
  && git clone -b 4.57.2 https://github.com/IntelRealSense/realsense-ros.git src/realsense_ros \   
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i
RUN /$WORKDIR/colcon_build.sh

# Install repo packages:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_realsense/src/ /$WORKDIR/ros/src
RUN /$WORKDIR/colcon_build.sh --symlink-install

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml /$WORKDIR/pyproject.toml
RUN uv sync \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

##############################
# Runtime
##############################

FROM ${BASE_IMAGE}

ENV ROS_DISTRO=jazzy

# Copy minimal dependencies
COPY --from=builder /$WORKDIR/external/src /$WORKDIR/external/src
WORKDIR /$WORKDIR/external
RUN apt update \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i \
  && rm -rf src \
  && rm -rf /var/lib/apt/lists/*

# Copy ROS install
COPY --from=builder /$WORKDIR/ros /$WORKDIR/ros
COPY --from=builder /$WORKDIR/external/install /$WORKDIR/external/install

# Copy environments
COPY --from=builder /$WORKDIR/.venv /$WORKDIR/.venv
COPY --from=builder /root/.bashrc /root/.bashrc

# Finalize
WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
