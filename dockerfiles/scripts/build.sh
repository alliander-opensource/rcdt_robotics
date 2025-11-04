#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

############################################################################################
# Script to build one of the non-base Dockerfiles.
# Each needs the following arguments:
#  - ARCH: architecture to build for [amd64 / arm64]
#  - PACKAGE: package name to build [core / franka / husarion / velodyne / vision]
#  - BASE_PACKAGE: base image tag [base / core]
############################################################################################

ARCH=$1
PACKAGE=$2
BASE_PACKAGE=$3
NO_CACHE=$4
export DOCKER_BUILDKIT=1

# Set Docker image tag
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
GIT_BRANCH=$(git branch --show-current)
if [ "$GIT_BRANCH" == "main" ] ; then
  IMAGE_TAG=$PACKAGE
else
  IMAGE_TAG=$PACKAGE-$GIT_BRANCH
fi

source ./checks.sh $ARCH

# Check if base image with specific branch tag exists, otherwise use latest
if [ -n "$(docker images | grep rcdt/robotics | grep $BASE_PACKAGE-$GIT_BRANCH-$ARCH)" ] ; then
  echo "Using base image with tag $BASE_PACKAGE-$GIT_BRANCH-$ARCH."
  BASE_IMAGE_GIT_TAG=$BASE_PACKAGE-$GIT_BRANCH-$ARCH
else
  BASE_IMAGE_GIT_TAG=$BASE_PACKAGE-$ARCH
  echo "Base image with tag $BASE_PACKAGE-$GIT_BRANCH-$ARCH not found. Using latest."
fi
echo $BASE_IMAGE_GIT_TAG
BASE_IMAGE=rcdt/robotics:$BASE_IMAGE_GIT_TAG
echo $BASE_IMAGE

# Build the Docker image
(
  cd "$SCRIPT_DIR"/.. && \
  docker build -f rcdt_$PACKAGE.Dockerfile \
  --build-arg BASE_IMAGE=$BASE_IMAGE \
  --platform $PLATFORM \
  $NO_CACHE \
  -t "rcdt/robotics:$IMAGE_TAG-$ARCH" \
  .
  )
