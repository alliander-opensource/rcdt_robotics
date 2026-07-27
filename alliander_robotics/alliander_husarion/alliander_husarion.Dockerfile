# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE AS builder

ARG SRC_DIRECTORY
ARG COLCON_BUILD_SEQUENTIAL
ENV ROS_DISTRO=jazzy

# Install ROS dependencies 
RUN apt update && apt install -y --no-install-recommends \
  ros-$ROS_DISTRO-twist-mux \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# Install Husarion packages
WORKDIR /$WORKDIR/external
RUN apt update \
  && git clone -b 2.3.1 https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros \
  && export HUSARION_ROS_BUILD_TYPE=simulation \ 
  && vcs import src < src/husarion_ugv_ros/husarion_ugv/${HUSARION_ROS_BUILD_TYPE}_deps.repos \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i \
  && . /opt/ros/$ROS_DISTRO/setup.sh \ 
  && colcon build \
  --packages-up-to \
  husarion_ugv \
  husarion_ugv_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \ 
  --event-handlers console_direct+
RUN /$WORKDIR/colcon_build.sh

# Install repo packages:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_husarion/src/ /$WORKDIR/ros/src
RUN /$WORKDIR/colcon_build.sh --symlink-install

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml /$WORKDIR/pyproject.toml
RUN uv sync  \
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
        ros-$ROS_DISTRO-twist-mux \
    && rm -rf /var/lib/apt/lists/*

# Copy minimal dependencies
COPY --from=builder /$WORKDIR/external/src /$WORKDIR/external/src
WORKDIR /$WORKDIR/external
RUN apt update \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i \
  # && rm -rf src \
  && rm -rf /var/lib/apt/lists/*

# Copy ROS install
COPY --from=builder /$WORKDIR/ros /$WORKDIR/ros
COPY --from=builder /$WORKDIR/external/install /$WORKDIR/external/install

# Copy Python environment
COPY --from=builder /$WORKDIR/.venv /$WORKDIR/.venv
ENV VIRTUAL_ENV=/$WORKDIR/.venv
ENV PATH="/$WORKDIR/.venv/bin:$PATH"
ENV PYTHONPATH="/$WORKDIR/.venv/lib/python3.12/site-packages"

RUN echo "source /$WORKDIR/external/install/setup.bash" >> /root/.bashrc && \
    echo "source /$WORKDIR/ros/install/setup.bash" >> /root/.bashrc

WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
