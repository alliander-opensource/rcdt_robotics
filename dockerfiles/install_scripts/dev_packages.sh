# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

# Specifying numpy range to work with ROS and Ultralytics:
pip install "numpy>=1.23.0,<2.0"

# Specifying currently newest version of transforms3d to avoid conflict with imported numpy.float in older version.
pip install "transforms3d>=0.4.2"