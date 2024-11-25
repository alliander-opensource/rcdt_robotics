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
RUN apt install -y ros-humble-realsense2-camera
RUN apt install -y ros-humble-realsense2-description

# rcdt_utilities
RUN apt install -y ros-humble-rviz-visual-tools