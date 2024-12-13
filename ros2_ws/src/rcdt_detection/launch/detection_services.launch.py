# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    define_centroid_node = Node(
        package="rcdt_detection", executable="define_centroid_node.py"
    )
    filter_masks_node = Node(
        package="rcdt_detection", executable="filter_masks_node.py"
    )
    get_rgbd_from_topic_node = Node(
        package="rcdt_detection", executable="get_rgbd_from_topic_node.py"
    )
    pose_from_pixel_node = Node(
        package="rcdt_detection", executable="pose_from_pixel_node.py"
    )
    publish_image_node = Node(
        package="rcdt_detection", executable="publish_image_node.py"
    )
    segment_image_node = Node(
        package="rcdt_detection", executable="segment_image_node.py"
    )
    split_rgbd_node = Node(package="rcdt_detection", executable="split_rgbd_node.py")

    return LaunchDescription(
        [
            define_centroid_node,
            filter_masks_node,
            get_rgbd_from_topic_node,
            pose_from_pixel_node,
            publish_image_node,
            segment_image_node,
            split_rgbd_node,
        ]
    )
