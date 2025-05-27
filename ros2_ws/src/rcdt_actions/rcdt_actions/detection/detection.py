# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from rcdt_messages.srv import GetRGBDFromTopic, SegmentImage

from rcdt_actions.definitions import Action


def get_rgbd_from_topic() -> Action:
    """Create an action to get RGBD data from a topic.

    Returns:
        Action: An action that can be used to get RGBD data from a topic.

    """
    return Action("/get_rgbd_from_topic", GetRGBDFromTopic)


def segment_image() -> Action:
    """Create an action to segment an image.

    Returns:
        Action: An action that can be used to segment an image.

    """
    return Action("/segment_image", SegmentImage)
