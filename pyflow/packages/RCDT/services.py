# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rcdt_detection_msgs.srv as detection
import rcdt_utilities_msgs.srv as utilities
from RCDT.Core.definitions import ServiceDefinition


unique_service_types: set[type] = set()
service_definitions: list[ServiceDefinition] = []


def add(service_name: str, service_type: type) -> None:
    unique_service_types.add(service_type)
    service_definitions.append(ServiceDefinition(service_name, service_type))


add("/define_centroid", detection.DefineCentroid)
add("/filter_masks", detection.FilterMasks)
add("/get_bounding_box_2d", detection.GetBoundingBox2D)
add("/get_largetst_contour", detection.GetLargestContour)
add("/get_mean_hue", detection.GetMeanHue)
add("/get_rgbd_from_topic", detection.GetRGBDFromTopic)
add("/get_rectangle_factor", detection.GetRectangleFactor)
add("/pose_from_pixel", detection.PoseFromPixel)
add("/publish_image", detection.PublishImage)
add("/segment_image", detection.SegmentImage)
add("/select_image_from_list", detection.SelectImageFromList)
add("/split_rgbd", detection.SplitRGBD)

add("/add_marker", utilities.AddMarker)
add("/add_object", utilities.AddObject)
add("/express_pose_in_other_frame", utilities.ExpressPoseInOtherFrame)
add("/is_value_betwee_limits", utilities.IsValueBetweenLimits)
add("/move_hand_to_pose", utilities.MoveHandToPose)
add("/move_to_configuration", utilities.MoveToConfiguration)
add("/transform_pose", utilities.TransformPose)
