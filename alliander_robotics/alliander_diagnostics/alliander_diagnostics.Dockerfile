# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARG BASE_IMAGE=ubuntu:latest
FROM $BASE_IMAGE 

ARG SRC_DIRECTORY
ARG COLCON_BUILD_SEQUENTIAL
ENV ROS_DISTRO=jazzy

# Install transitive:
RUN curl -sf "https://install.transitiverobotics.com?id=rewvm&token=551j2p5z5a3l1n2g5b32323u&docker=true" | bash

# Install webrtc dependencies:
# https://transitiverobotics.com/caps/transitive-robotics/webrtc-video/#dependencies
RUN apt update && apt install -y \
  build-essential \
  pkg-config \
  fontconfig \
  git \
  gobject-introspection \
  gstreamer1.0-x \
  gstreamer1.0-libav \
  gstreamer1.0-nice \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-base-apps \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools \
  libgstreamer1.0-0 \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  libgirepository1.0-dev \
  libc-dev \
  libcairo2 \
  libcairo2-dev

# Copy transitive start script:
COPY $SRC_DIRECTORY/alliander_diagnostics/transitive/start.sh /transitive/start.sh

# Install repo packages:
WORKDIR /$WORKDIR/ros
COPY $SRC_DIRECTORY/alliander_core/src/ /$WORKDIR/ros/src
COPY $SRC_DIRECTORY/alliander_diagnostics/src/ /$WORKDIR/ros/src
RUN /$WORKDIR/colcon_build.sh

# Install python dependencies:
WORKDIR $WORKDIR
COPY $SRC_DIRECTORY/pyproject.toml /$WORKDIR/pyproject.toml
RUN uv sync --group alliander-description \
  && echo "export PYTHONPATH=\"$(dirname $(dirname $(uv python find)))/lib/python3.12/site-packages:\$PYTHONPATH\"" >> /root/.bashrc \
  && echo "export PATH=\"$(dirname $(dirname $(uv python find)))/bin:\$PATH\"" >> /root/.bashrc

WORKDIR /$WORKDIR
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
