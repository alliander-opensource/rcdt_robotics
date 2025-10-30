#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

echo "Installing dependencies..."

# Make Ubuntu up-to-date including the latest stable kernel
echo "Updating Ubuntu..."
apt-get update -y && apt-get upgrade -y && apt-get dist-upgrade -y

# Install the core packages required to build librealsense binaries and the affected kernel modules
echo "Installing core packages..."
apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

# Cmake Note: certain librealsense CMAKE flags (e.g. CUDA) require version 3.8+ which is currently not made available via apt manager for Ubuntu LTS.

# Install build tools
echo "Installing build tools..."
apt-get install -y git wget cmake build-essential v4l-utils rsync

# Prepare Linux Backend and the Dev. Environment
# Unplug any connected RealSense camera and run:
echo "Please unplug any connected RealSense camera before proceeding."
echo "Installing backend packages..."
apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

echo "Dependencies installed. Proceeding with RealSense SDK installation..."

# URL of the zip file
ZIP_URL="https://github.com/IntelRealSense/librealsense/releases/download/v2.57.3/librealsense2_jammy_x86_debians_2_57_3_beta.zip"

# Temporary directory for download and extraction
TEMP_DIR="/tmp/realsense_install"
ZIP_FILE="$TEMP_DIR/librealsense2_jammy_x86_debians_2_57_3_beta.zip"

# Create temp directory
mkdir -p "$TEMP_DIR"

# Download the zip file
echo "Downloading RealSense SDK debs..."
wget -O "$ZIP_FILE" "$ZIP_URL"

# Unzip the file
echo "Extracting debs..."
unzip "$ZIP_FILE" -d "$TEMP_DIR"

# Find and install all .deb files
echo "Installing debs..."
dpkg -i "$TEMP_DIR"/*.deb

# Clean up
echo "Cleaning up..."
rm -rf "$TEMP_DIR"

echo "Installation completed successfully."