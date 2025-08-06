#!/usr/bin/env bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# install_zed.sh – non-interactive install of the ZED SDK on Ubuntu 22.04
# Mirrors the logic that was previously baked into the Dockerfile layer.

set -euo pipefail

### ──────────────────────────────────────────────────────────────
###  Customisable parameters (edit or export before running)
### ──────────────────────────────────────────────────────────────
: "${UBUNTU_RELEASE_YEAR:=22}"      # 20 → 20.04, 22 → 22.04, …
: "${CUDA_MAJOR:=11}"               # CUDA major version (11 or 12)
: "${CUDA_MINOR:=8}"                # CUDA minor version (e.g. 8 → 11.8)
: "${ZED_SDK_MAJOR:=5}"
: "${ZED_SDK_MINOR:=0}"             # Full SDK version is "${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}"
: "${TIMEZONE:=Europe/Paris}"       # Used only for the /etc/localtime line

### ──────────────────────────────────────────────────────────────
###  Sanity checks
### ──────────────────────────────────────────────────────────────
if   [[ $EUID -ne 0 ]]; then
  echo "Please run as root (e.g. sudo ./install_zed.sh)" >&2
  exit 1
fi

if ! command -v wget >/dev/null 2>&1; then
  apt-get update -y && apt-get install -y wget
fi

### ──────────────────────────────────────────────────────────────
###  CUDA “version.txt” marker (optional, cosmetic)
### ──────────────────────────────────────────────────────────────
echo "$TIMEZONE" > /etc/localtime
echo "CUDA Version ${CUDA_MAJOR}.${CUDA_MINOR}.0" > /usr/local/cuda/version.txt || true

### ──────────────────────────────────────────────────────────────
###  System prerequisites
### ──────────────────────────────────────────────────────────────
apt-get update -y \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       lsb-release wget less udev sudo zstd build-essential cmake \
       python3 python3-pip libpng-dev libgomp1

### ──────────────────────────────────────────────────────────────
###  Download + install ZED SDK (runtime, no tools, skip CUDA check)
### ──────────────────────────────────────────────────────────────
installer="ZED_SDK_Ubuntu${UBUNTU_RELEASE_YEAR}_cuda${CUDA_MAJOR}.${CUDA_MINOR}.run"
sdk_url="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}/ubuntu${UBUNTU_RELEASE_YEAR}"

echo "Downloading ${sdk_url}  →  ${installer} ..."
wget -q -O "${installer}" "${sdk_url}"
chmod +x "${installer}"

echo "Running installer …"
./"${installer}" -- silent skip_cuda

### ──────────────────────────────────────────────────────────────
###  Post-install tidy-up
### ──────────────────────────────────────────────────────────────
# libusb symlink occasionally required on bare-bones systems
ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0  /usr/lib/x86_64-linux-gnu/libusb-1.0.so

# make libsl_ai.so, libsl_zed.so etc. readable for all users
chmod 755 /usr/local/zed/lib
find /usr/local/zed/lib -type f -name 'libsl_*.so*' -exec chmod 644 {} +

rm -f "${installer}"
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "✅  ZED SDK ${ZED_SDK_MAJOR}.${ZED_SDK_MINOR} installed successfully."
echo "   • Libraries      : /usr/local/zed/lib"
echo "   • Udev rules     : run   sudo /usr/local/zed/tools/install_udev_rules.sh"
echo "   • Settings dir   : /usr/local/zed/settings  (must be writable)"