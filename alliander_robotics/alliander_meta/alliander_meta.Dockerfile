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

# Install adb:
RUN apt update && apt install -y android-tools-adb

# Install srcpy:
RUN apt update && apt install -y \
  ffmpeg libsdl2-2.0-0 adb wget gcc git pkg-config meson ninja-build libsdl2-dev libavcodec-dev \
  libavdevice-dev libavformat-dev libavutil-dev libswresample-dev libusb-1.0-0 libusb-1.0-0-dev \
  && git clone -b v3.3.4 https://github.com/Genymobile/scrcpy \
  && cd scrcpy \
  && ./install_release.sh

# Install ROS depenencies:
RUN apt update && apt install -y ros-$ROS_DISTRO-moveit-msgs

# Install repo packages:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_meta/src/ /$WORKDIR/ros/src
RUN /$WORKDIR/colcon_build.sh --symlink-install

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml/ /$WORKDIR/pyproject.toml
RUN uv sync --group alliander-meta \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

##############################
# Runtime stage
##############################

FROM ${BASE_IMAGE}

ENV ROS_DISTRO=jazzy

# Install minimal dependencies
RUN apt update && apt install -y \
  ffmpeg libsdl2-2.0-0 libusb-1.0-0 android-tools-adb \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

RUN apt update && apt install -y ros-$ROS_DISTRO-moveit-msgs

# Copy ROS install
COPY --from=builder /$WORKDIR/ros /$WORKDIR/ros

# Copy scrcpy installation
COPY --from=builder /usr/local/bin/scrcpy /usr/local/bin/scrcpy
COPY --from=builder /usr/local/share/scrcpy /usr/local/share/scrcpy

# Copy environments
COPY --from=builder /$WORKDIR/.venv /$WORKDIR/.venv
COPY --from=builder /root/.bashrc /root/.bashrc

# Finalize
WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
