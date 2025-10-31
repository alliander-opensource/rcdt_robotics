#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
# apt update
apt-get update  # && apt-get upgrade -y && apt-get dist-upgrade -y

# Core packages
apt-get install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev

# Build tools
apt-get install -y \
    git \
    wget \
    cmake \
    build-essential \
    v4l-utils \
    rsync

# Linux Backend and Dev. Environment
apt-get install -y \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at

echo "Dependencies installed. Installing the RealSense SDK..."

# Download + install RealSense SDK
ZIP_URL="https://github.com/IntelRealSense/librealsense/releases/download/v2.57.3/librealsense2_jammy_x86_debians_2_57_3_beta.zip"
TEMP_DIR="/tmp/realsense_install"
ZIP_FILE="$TEMP_DIR/librealsense2_jammy_x86_debians_2_57_3_beta.zip"

mkdir -p "$TEMP_DIR"
wget -O "$ZIP_FILE" "$ZIP_URL"
unzip "$ZIP_FILE" -d "$TEMP_DIR"
dpkg -i "$TEMP_DIR"/*.deb
rm -rf "$TEMP_DIR"

echo "Installation completed."