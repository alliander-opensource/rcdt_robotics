#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

mkdir -p /home/$UNAME/rosboard_ws/src
cd /home/$UNAME/rosboard_ws
git clone https://github.com/dheera/rosboard.git src/rosboard

source /home/$UNAME/.bashrc
colcon build
echo "source /home/$UNAME/rosboard_ws/install/setup.bash" >>/home/$UNAME/.bashrc
