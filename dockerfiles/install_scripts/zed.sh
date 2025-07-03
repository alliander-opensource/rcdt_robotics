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

# Setup the ZED SDK
RUN apt-get update -y || true ; apt-get install --no-install-recommends wget zstd udev libgomp1 -y ; \
    wget -O ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR} && \
    chmod +x ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run ; ./ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run -- silent runtime_only skip_cuda && \
    rm ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run && \
    rm -rf /var/lib/apt/lists/*