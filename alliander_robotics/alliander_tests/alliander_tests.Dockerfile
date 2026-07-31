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

# Install Docker CLI: 
RUN curl -fsSL https://get.docker.com | sh

# Install Doxygen:
RUN apt update && apt install -y --no-install-recommends \
    doxygen \
    && rm -rf /var/lib/apt/lists/* \
    && apt autoremove -y \
    && apt clean

# Install dependencies for ty:
RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-moveit-configs-utils \
    ros-$ROS_DISTRO-nav2-simple-commander \
    && rm -rf /var/lib/apt/lists/* \
    && apt autoremove -y \
    && apt clean

# Non-apt dependencies:
WORKDIR /$WORKDIR/external
RUN git clone --depth=1 --filter=blob:none -b v3.1.1 \
    https://github.com/frankarobotics/franka_ros2.git src/franka_ros2 \
    && cd src/franka_ros2 \
    && git sparse-checkout set franka_msgs
# Install newer version of launch_testing from source, to solve conflict with pytest: https://github.com/ros2/launch/pull/972
RUN git clone --depth=1 --filter=blob:none -b 3.10.0 \
    https://github.com/ros2/launch.git src/launch \
    && cd src/launch \
    && git sparse-checkout set launch_testing 
RUN /$WORKDIR/colcon_build.sh

# Install repo packages:
# alliander_seekthermal is needed as it creates a Python package that is imported
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
RUN /$WORKDIR/colcon_build.sh --symlink-install

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml /$WORKDIR/pyproject.toml
RUN uv sync --all-groups \
    && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
    && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

##############################
# Runtime stage
##############################

FROM ${BASE_IMAGE}

ENV ROS_DISTRO=jazzy

# Install minimal dependencies
RUN curl -fsSL https://get.docker.com | sh
RUN apt update && apt install -y --no-install-recommends \
    doxygen \
    ros-$ROS_DISTRO-moveit-configs-utils \
    ros-$ROS_DISTRO-nav2-simple-commander \
    && rm -rf /var/lib/apt/lists/* \
    && apt autoremove -y \
    && apt clean

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

WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
