# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

########################################################
### Dependencies required for our developed packages ###
########################################################

# rcdt_detection
RUN python3 -m pip install pyrealsense2
RUN python3 -m pip install ultralytics
RUN python3 -m pip install "numpy>=1.23.0,<2.0"
# specifying currently newest version of transforms3d to avoid conflict with imported numpy.float in older version.
RUN python3 -m pip install "transforms3d>=0.4.2"

# Realsense:
RUN apt update && apt install -y ros-humble-realsense2-camera
RUN apt update && apt install -y ros-humble-realsense2-description

# Velodyne:
RUN apt update && apt install -y ros-humble-velodyne-description

# Rosboard (https://github.com/dheera/rosboard):
RUN python3 -m pip install tornado
RUN python3 -m pip install simplejpeg 
RUN mkdir -p /home/$UNAME/rosboard_ws/src
WORKDIR /home/$UNAME/rosboard_ws
RUN git clone https://github.com/dheera/rosboard.git src/rosboard
RUN . /opt/ros/humble/setup.sh && colcon build
RUN echo "source /home/$UNAME/rosboard_ws/install/setup.bash" >> /home/$UNAME/.bashrc

# PyFlow (https://github.com/alliander-opensource/PyFlow):
RUN pip install git+https://github.com/alliander-opensource/PyFlow.git@master
RUN apt update && apt install -y libxcb-cursor-dev
RUN pip install inflection
RUN pip install distinctipy
ENV QT_API=pyside6

# Bio_ik (https://github.com/PickNikRobotics/bio_ik):
RUN mkdir -p /home/$UNAME/bio_ik_ws/src
WORKDIR /home/$UNAME/bio_ik_ws/src
RUN git clone https://github.com/PickNikRobotics/bio_ik.git -b ros2
WORKDIR /home/$UNAME/bio_ik_ws/
RUN . /opt/ros/humble/setup.sh && . /home/$UNAME/moveit_ws/install/setup.sh && colcon build
RUN echo "source /home/$UNAME/bio_ik_ws/install/setup.bash" >> /home/$UNAME/.bashrc

# Sphinx:
RUN pip install sphinx
RUN pip install sphinx-autobuild
RUN pip install myst-parser
RUN pip install sphinx_copybutton
RUN pip install sphinx_rtd_theme
RUN pip install sphinxcontrib-video

# moveit_visual_tools
RUN mkdir -p /home/$UNAME/moveit_visual_tools/src
WORKDIR /home/$UNAME/moveit_visual_tools/src
RUN git clone -b ros2 https://github.com/ros-planning/moveit_visual_tools
RUN vcs import < moveit_visual_tools/moveit_visual_tools.repos
WORKDIR /home/$UNAME/moveit_visual_tools
RUN . /opt/ros/humble/setup.sh && . /home/$UNAME/moveit_ws/install/setup.sh && \ 
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source /home/$UNAME/moveit_visual_tools/install/setup.bash" >> /home/$UNAME/.bashrc

# General python packages:
RUN pip install mashumaro