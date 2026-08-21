# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE AS builder

##############################
# Build stage
##############################

ARG SRC_DIRECTORY
ENV ROS_DISTRO=jazzy

# Install srcpy:
RUN apt update && apt install -y \
  ffmpeg libsdl2-2.0-0 adb wget gcc git pkg-config meson ninja-build libsdl2-dev libavcodec-dev \
  libavdevice-dev libavformat-dev libavutil-dev libswresample-dev libusb-1.0-0 libusb-1.0-0-dev \
  && git clone -b v3.3.4 https://github.com/Genymobile/scrcpy \
  && cd scrcpy \
  && ./install_release.sh

# Install alliander packages:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_meta/src/ /$WORKDIR/ros/src
RUN --mount=type=cache,id=apt-cache,target=/var/cache/apt,sharing=locked --mount=type=cache,id=apt-lists,target=/var/lib/apt,sharing=locked /$WORKDIR/rosdep_install.sh --build
RUN /$WORKDIR/colcon_build.sh

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml/ /$WORKDIR/pyproject.toml
RUN --mount=type=cache,id=uv-cache,target=/root/.cache/uv uv sync --group alliander-meta \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

##############################
# Runtime stage
##############################

FROM ${BASE_IMAGE}
ENV ROS_DISTRO=jazzy

# Copy environments
COPY --from=builder /$WORKDIR/.venv /$WORKDIR/.venv
COPY --from=builder /root/.bashrc /root/.bashrc
COPY --from=builder /usr/local/bin/scrcpy /usr/local/bin/scrcpy
COPY --from=builder /usr/local/share/scrcpy /usr/local/share/scrcpy

# Install adb:
RUN --mount=type=cache,id=apt-cache,target=/var/cache/apt,sharing=locked --mount=type=cache,id=apt-lists,target=/var/lib/apt,sharing=locked \
  apt update && apt install -y --no-install-recommends android-tools-adb

# Copy alliander packages and install runtime dependencies:
WORKDIR /$WORKDIR/ros
COPY --from=builder /$WORKDIR/ros /$WORKDIR/ros
RUN --mount=type=cache,id=apt-cache,target=/var/cache/apt,sharing=locked --mount=type=cache,id=apt-lists,target=/var/lib/apt,sharing=locked /$WORKDIR/rosdep_install.sh --exec

# Finalize
WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
