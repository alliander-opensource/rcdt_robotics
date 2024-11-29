# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Pull the base image:
FROM osrf/ros:humble-desktop
ARG COLCON_BUILD_SEQUENTIAL
ENV UNAME=rcdt
ENV UID=1000
