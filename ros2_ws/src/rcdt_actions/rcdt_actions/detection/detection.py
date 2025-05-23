# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from rcdt_messages.srv import GetRGBDFromTopic, SegmentImage

from rcdt_actions.definitions import Action


def get_rgbd_from_topic() -> Action:
    return Action("/get_rgbd_from_topic", GetRGBDFromTopic)


def segment_image() -> Action:
    return Action("/segment_image", SegmentImage)
