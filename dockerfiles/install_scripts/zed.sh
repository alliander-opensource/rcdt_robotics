#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

ARG UBUNTU_RELEASE_YEAR=22
ARG CUDA_MAJOR=12
ARG CUDA_MINOR=8

ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=0

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility

# 1. refresh APT metadata
RUN apt-get update -y || true

# 2. install runtime dependencies
RUN apt-get install --no-install-recommends -y \
        wget zstd udev libgomp1

# 3. download the ZED SDK installer
RUN wget --content-disposition \
        https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR}