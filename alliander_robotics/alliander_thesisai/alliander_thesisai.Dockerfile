# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE

ARG COLCON_BUILD_SEQUENTIAL
ENV ROS_DISTRO=jazzy

# Install ROS dependencies
# ToDo: Check whether I need rosdep update / install here 
RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-moveit-msgs \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

WORKDIR /$WORKDIR/ros
COPY alliander_robotics/alliander_core/src/ /$WORKDIR/ros/src
COPY alliander_robotics/alliander_thesisai/src/ /$WORKDIR/ros/src
RUN apt update && rosdep update --rosdistro $ROS_DISTRO && rosdep install --from-paths /$WORKDIR/ros/src -y -i -t build
RUN /$WORKDIR/colcon_build.sh

# Install repo packages:
# WORKDIR /$WORKDIR/ros
# COPY alliander_robotics/alliander_core/src/ /$WORKDIR/ros/src
# COPY alliander_robotics/alliander_openvla/src/ /$WORKDIR/ros/src

# RUN rosdep update --rosdistro $ROS_DISTRO \
#   && rosdep install --from-paths src -y -i

# RUN /$WORKDIR/colcon_build.sh

# Install python dependencies:
WORKDIR $WORKDIR
COPY pyproject.toml /$WORKDIR/pyproject.toml
# Only here should it stay connected thesisAI not thesisai
RUN uv sync --all-groups \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

# Finalize
WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
