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

# Install L3Cam SDK:
ARG TEMP_DIR="/tmp/beamagine_install"
ARG RUN_FILE="$TEMP_DIR/l3cam_sdk.run"
RUN mkdir -p "$TEMP_DIR"
RUN if [ $(dpkg --print-architecture) = "amd64" ]; \
  then wget -O "$RUN_FILE" "https://github.com/beamaginelidar/libl3cam/releases/download/0.2.1R/libl3cam_0.2.1-1_amd64.deb"; \
  elif [ $(dpkg --print-architecture) = "arm64" ]; \
  then wget -O "$RUN_FILE" "https://github.com/beamaginelidar/libl3cam/releases/download/0.2.1R/libl3cam_0.2.1-1_arm64.deb"; \
  else echo "Unsupported architecture: $(dpkg --print-architecture)"; exit 1; fi
RUN dpkg -i "${RUN_FILE}"

# Set buffer size:
RUN echo 'net.core.rmem_default=268435456' >> /etc/sysctl.conf && \
  echo 'net.core.rmem_max=268435456' >> /etc/sysctl.conf && \
  echo 'net.core.netdev_max_backlog=5000' >> /etc/sysctl.conf

# Install ROS driver:
WORKDIR /$WORKDIR/external
RUN apt update \
  && git clone -b 1.0.3 https://github.com/beamaginelidar/l3cam_ros2.git src/l3cam_ros2 \
  && cd /$WORKDIR/external \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i
RUN /$WORKDIR/colcon_build.sh

# Install repo package:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_beamagine/src/ /$WORKDIR/ros/src
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
COPY --from=builder /etc/sysctl.conf /etc/sysctl.conf

# Finalize
WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
