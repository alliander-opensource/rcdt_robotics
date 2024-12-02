# syntax = devthefuture/dockerfile-x
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

INCLUDE ./general/pre_install.dockerfile

INCLUDE ./general/dep_core.dockerfile

INCLUDE ./general/moveit.dockerfile

INCLUDE ./panther/main.dockerfile

INCLUDE ./general/post_install.dockerfile