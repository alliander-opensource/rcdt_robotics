# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Pull the base image:
FROM osrf/ros:humble-desktop
ARG COLCON_BUILD_SEQUENTIAL
ENV UNAME=rcdt
ENV UID=1000

# Remove existing user with same UID if exists:
RUN if getent passwd $UID; then userdel $(id -nu $UID); else :; fi

# Add user:
RUN useradd -m -u $UID -p "$(openssl passwd -1 $UNAME)" $UNAME
RUN usermod -aG sudo,dialout $UNAME
    
# Update image:
RUN apt update
RUN apt full-upgrade -y

# Set .bashrc:
COPY ./bashrc/.bashrc /home/$UNAME/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$UNAME/.bashrc
