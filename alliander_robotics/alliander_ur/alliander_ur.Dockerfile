# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG COLCON_BUILD_SEQUENTIAL
ENV ROS_DISTRO=jazzy

# Install UR packages
RUN apt update && apt install -y --no-install-recommends \
  ros-$ROS_DISTRO-ur \
  ros-$ROS_DISTRO-ur-simulation-gz \
  ros-$ROS_DISTRO-ur-description \
  ros-$ROS_DISTRO-ur-moveit-config \
  ros-$ROS_DISTRO-ur-robot-driver \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# Install repo packages:
WORKDIR /$WORKDIR/ros
COPY alliander_robotics/alliander_core/src/ /$WORKDIR/ros/src
COPY alliander_robotics/alliander_arm_controllers/src/ /$WORKDIR/ros/src
COPY alliander_robotics/alliander_ur/src/ /$WORKDIR/ros/src
RUN /$WORKDIR/colcon_build.sh

# Install python dependencies:
WORKDIR $WORKDIR
COPY pyproject.toml /$WORKDIR/pyproject.toml
RUN uv sync --group alliander-ur  \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
