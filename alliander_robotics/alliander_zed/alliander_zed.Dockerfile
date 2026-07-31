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

# Install ZED Wrapper:
WORKDIR /$WORKDIR/external
RUN apt update \
  && git clone -b v5.4.0 https://github.com/stereolabs/zed-ros2-wrapper.git src/zed_ros2_wrapper \
  && rm -rf src/zed_ros2_wrapper/zed_debug \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i
RUN /$WORKDIR/colcon_build.sh

# Install repo packages:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_zed/src/ /$WORKDIR/ros/src
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

# Install minimal dependencies
RUN apt update && \
    apt install -y \
        libturbojpeg0-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /$WORKDIR/external/src /$WORKDIR/external/src
WORKDIR /$WORKDIR/external
RUN apt update \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i \
  && rm -rf src \
  && rm -rf /var/lib/apt/lists/*

# Copy runtime SDK
COPY --from=builder /usr/local/zed /usr/local/zed
RUN echo "/usr/local/zed/lib" > /etc/ld.so.conf.d/zed.conf && ldconfig

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
