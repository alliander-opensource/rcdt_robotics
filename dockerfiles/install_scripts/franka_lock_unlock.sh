#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

mkdir -p /home/$UNAME/franka_lock_unlock/src
cd /home/$UNAME/franka_lock_unlock
git clone https://github.com/alliander-opensource/franka_lock_unlock.git

rosdep update
rosdep install --from-paths src --rosdistro humble -y -i

source /home/$UNAME/.bashrc
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source /home/$UNAME/franka_lock_unlock/install/setup.bash" >>/home/$UNAME/.bashrc
