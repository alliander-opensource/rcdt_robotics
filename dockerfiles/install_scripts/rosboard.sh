# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

pip install \
    tornado \
    simplejpeg
mkdir -p /home/$UNAME/rosboard_ws/src
cd /home/$UNAME/rosboard_ws
git clone https://github.com/dheera/rosboard.git src/rosboard

. /opt/ros/humble/setup.sh
colcon build
echo "source /home/$UNAME/rosboard_ws/install/setup.bash" >>/home/$UNAME/.bashrc
