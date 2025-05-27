# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import geometry_msgs.msg as geometry
import moveit_msgs.msg as moveit

messages: set[type] = set()


def add(message: type) -> None:
    """Add a message type to the set of messages.

    Args:
        message (type): The message type to add.
    """
    messages.add(message)


add(geometry.Transform)
add(geometry.PoseStamped)
add(moveit.CollisionObject)
