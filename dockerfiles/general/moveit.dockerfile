# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

RUN mkdir -p /home/$UNAME/moveit_ws/src
WORKDIR /home/$UNAME/moveit_ws/src

RUN git clone https://github.com/moveit/moveit2.git -b main
RUN for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
RUN rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

WORKDIR /home/$UNAME/moveit_ws/
RUN . /opt/ros/humble/setup.sh && colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release ${COLCON_BUILD_SEQUENTIAL:+--executor sequential}
RUN echo "source /home/$UNAME/moveit_ws/install/setup.bash" >> /home/$UNAME/.bashrc