# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Install panther_ros: https://github.com/husarion/panther_ros
WORKDIR /home/$UNAME
RUN mkdir husarion_ws
WORKDIR /home/$UNAME/husarion_ws
RUN git clone -b ros2 https://github.com/husarion/panther_ros.git src/panther_ros
ENV HUSARION_ROS_BUILD_TYPE=simulation
RUN vcs import src < src/panther_ros/panther/panther_$HUSARION_ROS_BUILD_TYPE.repos

RUN cp -r src/ros2_controllers/diff_drive_controller src
RUN cp -r src/ros2_controllers/imu_sensor_broadcaster src
RUN rm -rf src/ros2_controllers

RUN rosdep update --rosdistro $ROS_DISTRO
RUN rosdep install --from-paths src -y -i

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-up-to panther --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /home/$UNAME/husarion_ws/install/setup.bash" >> /home/$UNAME/.bashrc