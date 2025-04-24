# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

# Install pinocchio:
apt install -y lsb-release curl
mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list
apt update
apt install -y robotpkg-pinocchio

# Install libfranka: https://github.com/frankaemika/libfranka/tree/0.15.0
apt update
apt install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev

cd /home/$UNAME
git clone --recurse-submodules -b 0.15.0 https://github.com/frankaemika/libfranka.git
cd /home/$UNAME/libfranka
mkdir build
cd /home/$UNAME/libfranka/build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
make
cpack -G DEB
dpkg -i libfranka*.deb
