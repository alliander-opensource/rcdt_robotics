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

# Install ZED SDK:
ARG TEMP_DIR="/tmp/zed_install"
ARG RUN_FILE="$TEMP_DIR/zed_sdk.run"
RUN mkdir -p "$TEMP_DIR"
RUN if [ $(dpkg --print-architecture) = "amd64" ]; \
  then wget -O "$RUN_FILE" "https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.4/ZED_SDK_Ubuntu24_cuda13.0_tensorrt10.13_v5.4.0.zstd.run"; \
  elif [ $(dpkg --print-architecture) = "arm64" ]; \ 
  then wget -O "$RUN_FILE" "https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.4/ZED_SDK_Tegra_L4T39.2_v5.4.0.zstd.run"; \
  else echo "Unsupported architecture: $(dpkg --print-architecture)"; exit 1; fi
RUN chmod +x "${RUN_FILE}" \
  && "${RUN_FILE}" -- silent runtime_only skip_tools \
  && chmod -R u+rwX,go+rX /usr/local/zed \
  && rm -f "${RUN_FILE}" \
  && rm -rf /var/lib/apt/lists/*

# Install ZED description and msgs packages on specific version
RUN if [ "$(dpkg --print-architecture)" = "amd64" ]; then \
  apt update && apt install -y --no-install-recommends \
  ros-$ROS_DISTRO-zed-description=0.1.5-1noble.20260615.180130 \
  ros-$ROS_DISTRO-zed-msgs=5.3.0-1noble.20260615.112916; \
  elif [ "$(dpkg --print-architecture)" = "arm64" ]; then \
  apt update && apt install -y --no-install-recommends \
  ros-$ROS_DISTRO-zed-description=0.1.5-1noble.20260615.094525 \
  ros-$ROS_DISTRO-zed-msgs=5.3.0-1noble.20260612.085538; \
  else \
  echo "Unsupported architecture: $(dpkg --print-architecture)" && exit 1; \
  fi && \
  rm -rf /var/lib/apt/lists/* && \
  apt autoremove -y && \
  apt clean

# Install external packages:
WORKDIR /$WORKDIR/external
RUN git clone -b v5.4.0 https://github.com/stereolabs/zed-ros2-wrapper.git src/zed_ros2_wrapper \
  && rm -rf src/zed_ros2_wrapper/zed_debug
RUN --mount=type=cache,id=apt-cache,target=/var/cache/apt,sharing=locked --mount=type=cache,id=apt-lists,target=/var/lib/apt,sharing=locked /$WORKDIR/rosdep_install.sh --build
RUN /$WORKDIR/colcon_build.sh

# Install alliander packages:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_zed/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/utilities/depth_camera/ /$WORKDIR/ros/src/depth_camera
RUN --mount=type=cache,id=apt-cache,target=/var/cache/apt,sharing=locked --mount=type=cache,id=apt-lists,target=/var/lib/apt,sharing=locked /$WORKDIR/rosdep_install.sh --build
RUN /$WORKDIR/colcon_build.sh --symlink-install

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml /$WORKDIR/pyproject.toml
RUN --mount=type=cache,id=uv-cache,target=/root/.cache/uv uv sync \
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

# Install minimal dependencies
RUN apt update && apt install -y \
  libturbojpeg0-dev \
  && rm -rf /var/lib/apt/lists/*

# Copy runtime SDK
COPY --from=builder /usr/local/zed /usr/local/zed
RUN echo "/usr/local/zed/lib" > /etc/ld.so.conf.d/zed.conf && ldconfig

# Copy external packages and install runtime dependencies:
WORKDIR /$WORKDIR/external
COPY --from=builder /$WORKDIR/external /$WORKDIR/external
RUN --mount=type=cache,id=apt-cache,target=/var/cache/apt,sharing=locked --mount=type=cache,id=apt-lists,target=/var/lib/apt,sharing=locked /$WORKDIR/rosdep_install.sh --exec
RUN rm -rf src build log

# Copy alliander packages and install runtime dependencies:
WORKDIR /$WORKDIR/ros
COPY --from=builder /$WORKDIR/ros /$WORKDIR/ros
RUN --mount=type=cache,id=apt-cache,target=/var/cache/apt,sharing=locked --mount=type=cache,id=apt-lists,target=/var/lib/apt,sharing=locked /$WORKDIR/rosdep_install.sh --exec

# Finalize
WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
