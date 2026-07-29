# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG SRC_DIRECTORY
ARG COLCON_BUILD_SEQUENTIAL
ENV ROS_DISTRO=jazzy

# Install ROS dependencies
WORKDIR /$WORKDIR/external
RUN apt update \
  && apt install -y ros-$ROS_DISTRO-behaviortree-cpp \
  && git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git src/behaviortree_ros2 \
  && rosdep update --rosdistro $ROS_DISTRO \
  && rosdep install --from-paths src -y -i
RUN /$WORKDIR/colcon_build.sh

# Install repo packages:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_behaviortree/src/ /$WORKDIR/ros/src
RUN /$WORKDIR/colcon_build.sh

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml /$WORKDIR/pyproject.toml
RUN uv sync \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinit"]
