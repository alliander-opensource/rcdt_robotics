#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
ARCH=$1

export DOCKER_BUILDKIT=1

# Set Docker image tag
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
GIT_BRANCH=$(git branch --show-current)
if [ "$GIT_BRANCH" == "main" ] ; then
  IMAGE_TAG=latest
else
  IMAGE_TAG=$GIT_BRANCH
fi

source ./checks.sh $ARCH

# Check if base image with specific branch tag exists, otherwise use latest
if [ -n "$(docker images | grep rcdt/robotics-base | grep $IMAGE_TAG-$ARCH)" ] ; then
  echo "Using base image with tag $IMAGE_TAG."
  BASE_IMAGE_GIT_TAG=$IMAGE_TAG
else
  BASE_IMAGE_GIT_TAG=latest
  echo "Base image with tag $IMAGE_TAG not found. Using latest."
fi
BASE_IMAGE=rcdt/robotics-base:$BASE_IMAGE_GIT_TAG-$ARCH

# Build the Docker image
(
  cd "$SCRIPT_DIR"/.. && \
  docker build -f rcdt_robotics.Dockerfile \
  --build-arg BASE_IMAGE=$BASE_IMAGE \
  --platform $PLATFORM \
  -t "rcdt/robotics:$IMAGE_TAG-$ARCH" \
  .
  )
