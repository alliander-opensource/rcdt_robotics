# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG COLCON_BUILD_SEQUENTIAL
ENV ROS_DISTRO=jazzy

# Install ROS dependencies
#RUN apt update && apt install -y ros-$ROS_DISTRO-moveit-msgs

# Install ROS depenencies and upgrade conflicting packages to latest versions:
# OR use apt install -y --upgrade-only ? 
# ros-$ROS_DISTRO-fastcdr \
#     ros-$ROS_DISTRO-rosidl-typesupport-fastrtps-c \
RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-moveit-msgs \
  && rm -rf /var/lib/apt/lists/* \
  && apt autoremove -y \
  && apt clean

# RUN apt update \
# # 2. Install standard moveit-msgs package
#     && apt install -y ros-$ROS_DISTRO-moveit-msgs \
# # 3. Force-upgrade the extended middleware stack to match the new ABI
#     && apt install -y --only-upgrade \
#         ros-$ROS_DISTRO-fastcdr \
#         ros-$ROS_DISTRO-fastrtps \
#         ros-$ROS_DISTRO-rmw-fastrtps-cpp \
#         ros-$ROS_DISTRO-rmw-fastrtps-shared-cpp \
#         ros-$ROS_DISTRO-rosidl-typesupport-fastrtps-c \
#         ros-$ROS_DISTRO-rosidl-typesupport-fastrtps-cpp \
#         ros-$ROS_DISTRO-rclpy \
#         ros-$ROS_DISTRO-moveit-msgs \
# # 4. Clean cache
#     && rm -rf /var/lib/apt/lists/*


# Install repo packages:
WORKDIR /$WORKDIR/ros
COPY alliander_robotics/alliander_core/src/ /$WORKDIR/ros/src
COPY alliander_robotics/alliander_openvla/src/ /$WORKDIR/ros/src
RUN /$WORKDIR/colcon_build.sh

# Install python dependencies:
WORKDIR $WORKDIR
COPY pyproject.toml /$WORKDIR/pyproject.toml
RUN uv sync --group alliander-openvla \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

# Finalize
WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
