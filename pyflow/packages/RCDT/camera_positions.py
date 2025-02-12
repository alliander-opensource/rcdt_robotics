# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from rcdt_utilities.geometry import Point3D, Pose, Quaternion

POSITIONS = {
    "home": Pose(
        position=Point3D(1, -0.3, 0.5),
        orientation=Quaternion.from_eulerangles_deg(0, 10, 150),
    ),
    "front": Pose(
        position=Point3D(1.2, 0, 0.5),
        orientation=Quaternion.from_eulerangles_deg(0, 0, 180),
    ),
    "front-low": Pose(
        position=Point3D(0.7, 0, 0.15),
        orientation=Quaternion.from_eulerangles_deg(0, 0, 180),
    ),
    "top-down": Pose(
        position=Point3D(0, 0, 1.5),
        orientation=Quaternion.from_eulerangles_deg(0, 90, 180),
    ),
}
