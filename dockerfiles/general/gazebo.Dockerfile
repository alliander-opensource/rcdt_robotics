# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


# Instead of using https://gazebosim.org/docs/fortress/ros_installation/, 
# we install ignition separately first, to obtain a newer version. Originally 
# to have this issue fixed: https://github.com/gazebosim/gz-sensors/issues/454.


# Install gazebo: https://gazebosim.org/docs/fortress/install_ubuntu/
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress

# Install ros-humble-ros-gz (which will now use the already installed ignition):
RUN apt install -y ros-humble-ros-gz
RUN apt install -y ros-humble-ign-ros2-control