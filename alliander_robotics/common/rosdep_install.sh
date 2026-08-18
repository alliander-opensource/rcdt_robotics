#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Since this script is interactive, the .bashrc is sourced automatically!
# Therefore, packages that are build before and added to the .bashrc are automatically sourced.

set -e

 apt update && rosdep update --rosdistro $ROS_DISTRO

ARGS=()
if [ "$1" = "--build" ]; then
    rosdep install --from-paths src -y -i -t build
elif [ "$1" = "--exec" ]; then
    rosdep install --from-paths src -y -i -t exec
else
    echo "Invalid argument: $1"
    echo "Usage: $0 [--build|--exec]"
    exit 1
fi
