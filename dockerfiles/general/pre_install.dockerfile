# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Pull the base image:
FROM nvidia/cuda:12.9.1-devel-ubuntu24.04 AS base
ARG COLCON_BUILD_SEQUENTIAL
ENV UNAME=rcdt
ENV UID=1000

# Remove existing user with same UID if exists:
RUN if getent passwd $UID; then userdel $(id -nu $UID); else :; fi

# Add user:
RUN useradd -m -u $UID -p "$(openssl passwd -1 $UNAME)" $UNAME
RUN usermod -aG sudo,dialout $UNAME

# Update apt:
RUN apt update

RUN sudo rosdep init || true && rosdep update

# Create bashrc file:
RUN echo "if test -f ~/.personal.bashrc; then\nsource ~/.personal.bashrc\nfi" >> /home/$UNAME/.bashrc
RUN echo "if test -f ~/.env; then\nset -a && source ~/.env && set +a\nfi" >> /home/$UNAME/.bashrc