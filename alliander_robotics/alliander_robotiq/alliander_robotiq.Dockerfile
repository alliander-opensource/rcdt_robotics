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

# Install required packages:
RUN apt update && apt install -y --no-install-recommends \
  ros-$ROS_DISTRO-controller-manager \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# Upgrade CMake:
RUN pip install --upgrade cmake --break-system-packages

# Install repo package:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_robotiq/src/ /$WORKDIR/ros/src
RUN apt update && rosdep update --rosdistro $ROS_DISTRO && rosdep install --from-paths src -y -i
RUN /$WORKDIR/colcon_build.sh --symlink-install

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml /$WORKDIR/pyproject.toml
RUN uv sync --group alliander-robotiq  \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

##############################
# Runtime stage
##############################

FROM ${BASE_IMAGE}

ENV ROS_DISTRO=jazzy

# Install minimal dependencies
RUN apt update && apt install -y --no-install-recommends \
  ros-$ROS_DISTRO-controller-manager \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# Upgrade CMake:
RUN pip install --upgrade cmake --break-system-packages

# Copy ROS install
COPY --from=builder /$WORKDIR/ros /$WORKDIR/ros
RUN apt update && rosdep update --rosdistro $ROS_DISTRO && rosdep install --from-paths /$WORKDIR/ros/src -y -i

# Copy environments
COPY --from=builder /$WORKDIR/.venv /$WORKDIR/.venv
COPY --from=builder /root/.bashrc /root/.bashrc

# Finalize
WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
