# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Install libfranka:
INCLUDE ./libfranka.dockerfile

#Install ros2 franka:
INCLUDE ./franka_ros2.dockerfile