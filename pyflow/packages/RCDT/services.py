# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rcdt_detection_msgs.srv as detection
import rcdt_utilities_msgs.srv as utilities
from std_srvs.srv import Trigger
from RCDT.Core.definitions import ServiceDefinition


unique_service_types: set[type] = set()
service_definitions: list[ServiceDefinition] = []


def add(
    pyflow_name: str, service_name: str, service_type: type, pyflow_group: str = None
) -> None:
    unique_service_types.add(service_type)
    service_definitions.append(
        ServiceDefinition(service_name, service_type, pyflow_name, pyflow_group)
    )


# fmt: off
group = "Detection"
add("GetMaskProperties","/get_mask_properties", detection.GetMaskProperties, group)
add("GetRGBDFromTopic", "/get_rgbd_from_topic", detection.GetRGBDFromTopic, group)
add("PoseFromPixel", "/pose_from_pixel", detection.PoseFromPixel, group)
add("PublishImage", "/publish_image", detection.PublishImage, group)
add("PublishMasks", "/publish_masks", detection.PublishMasks, group)
add("SegmentImage", "/segment_image", detection.SegmentImage, group)
add("SelectImageFromList", "/select_image_from_list", detection.SelectImageFromList, group)
add("SelectPickLocation", "/select_pick_location", detection.SelectPickLocation, group)

group = "Utilities"
add("AddMarker", "/add_marker", utilities.AddMarker, group)
add("ExpressPoseInOtherFrame", "/express_pose_in_other_frame", utilities.ExpressPoseInOtherFrame, group)
add("TransformPose", "/transform_pose", utilities.TransformPose, group)

group = "moveit_controller"
add("AddObject", f"{group}/add_object", utilities.AddObject, group)
add("MoveHandToPose", f"{group}/move_hand_to_pose", utilities.MoveHandToPose, group)
add("MoveToConfiguration", f"{group}/move_to_configuration", utilities.MoveToConfiguration, group)
add("PickAtPose", f"{group}/pick_at_pose", utilities.MoveHandToPose, group)
add("Drop", f"{group}/drop", Trigger, group)

group = "Gripper"
add("OpenGripper", "/open_gripper", Trigger, group)
add("CloseGripper", "/close_gripper", Trigger, group)
