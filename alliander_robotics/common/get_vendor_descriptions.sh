# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
#!/bin/bash
set -e

runtime_install=false

for arg in "$@"; do
    if [ "$arg" = "--runtime-install" ]; then
        runtime_install=true
        break
    fi
done

if [ "$runtime_install" = false ]; then
    echo "Retrieving external source packages"
    # Franka
    git clone -b 2.1.0 https://github.com/frankarobotics/franka_description.git src/franka_description
    echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/install/franka_description/share" >> /root/.bashrc

    # Husarion
    git clone --depth=1 --filter=blob:none --sparse -b ros2 \
    https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros
    cd src/husarion_ugv_ros
    git sparse-checkout set husarion_ugv_description 
    cd ../..

    # Realsense
    git clone --depth=1 --filter=blob:none --sparse -b 4.57.2 \
    https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros
    cd src/realsense-ros 
    git sparse-checkout set realsense2_description
    cd ../..
else
    echo "Skipping external source packages (--runtime-install enabled)"
fi

# echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$alliander/ros/install/alliander_description/share" >> /root/.bashrc

# Architecture-dependent versioning
if [ "$(dpkg --print-architecture)" = "amd64" ]; then
    ZED_VERSION="0.1.5-1noble.20260615.180130"
elif [ "$(dpkg --print-architecture)" = "arm64" ]; then
    ZED_VERSION="0.1.5-1noble.20260615.094525"
else
    echo "Unsupported architecture: $(dpkg --print-architecture)"
    exit 1
fi

# Install dependencies
apt update && apt install -y --no-install-recommends \
    ros-$ROS_DISTRO-husarion-components-description \
    ros-$ROS_DISTRO-velodyne-description \
    ros-$ROS_DISTRO-zed-description=$ZED_VERSION \
    ros-$ROS_DISTRO-ewellix-description