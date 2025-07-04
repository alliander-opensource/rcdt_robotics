#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# This script installs the ZED SDK 5.0.3 on a Docker image based on Ubuntu 22.04 with CUDA 12.8.

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility

# 1. refresh APT metadata
RUN apt-get update -y || true

# 2. install runtime dependencies
RUN apt-get install --no-install-recommends -y \
        wget zstd udev libgomp1

# 3. download the ZED SDK installer
RUN wget --content-disposition \
        https://download.stereolabs.com/zedsdk/12.0/cu12/ubuntu22

RUN chmod +x ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.0.3.zstd.run && \
    ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.0.3.zstd.run silent skip_tools skip_cuda