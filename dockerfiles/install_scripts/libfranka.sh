#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

#Install libfranka: https://support.franka.de/docs/installation_linux.html#build-libfranka
apt update
apt install -y \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev

cd /home/$UNAME
git clone --recursive https://github.com/frankaemika/libfranka
cd /home/$UNAME/libfranka
git checkout 0.13.3
git submodule update
mkdir build
cd /home/$UNAME/libfranka/build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
cpack -G DEB
dpkg -i libfranka*.deb