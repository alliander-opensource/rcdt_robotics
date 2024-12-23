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
    get_bounding_box_2d_node = Node(
        package="rcdt_detection", executable="get_bounding_box_2d_node.py"
    )
    get_largest_contour_node = Node(
        package="rcdt_detection", executable="get_largest_contour_node.py"
    )
    get_mask_properties = Node(
        package="rcdt_detection", executable="get_mask_properties.py"
    )
    get_mean_hue_node = Node(
        package="rcdt_detection", executable="get_mean_hue_node.py"
    )
    get_rectangle_factor_node = Node(
        package="rcdt_detection", executable="get_rectangle_factor_node.py"
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
    select_image_from_list_node = Node(
        package="rcdt_detection", executable="select_image_from_list_node.py"
    )
    split_rgbd_node = Node(package="rcdt_detection", executable="split_rgbd_node.py")

    return LaunchDescription(
        [
            define_centroid_node,
            filter_masks_node,
            get_bounding_box_2d_node,
            get_largest_contour_node,
            get_mask_properties,
            get_mean_hue_node,
            get_rectangle_factor_node,
            get_rgbd_from_topic_node,
            pose_from_pixel_node,
            publish_image_node,
            segment_image_node,
            select_image_from_list_node,
            split_rgbd_node,
        ]
    )
