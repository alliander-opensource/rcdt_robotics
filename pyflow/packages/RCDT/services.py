# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from rcdt_messages import srv
from std_srvs.srv import Trigger

from RCDT.Core.definitions import ServiceDefinition

unique_service_types: set[type] = set()
service_definitions: list[ServiceDefinition] = []


def add(
    pyflow_name: str, service_name: str, service_type: type, pyflow_group: str = None
) -> None:
    """Add a service definition to the list of service definitions.

    Args:
        pyflow_name (str): The name of the service in PyFlow.
        service_name (str): The name of the service as it will be used in ROS.
        service_type (type): The type of the service.
        pyflow_group (str, optional): The group to which the service belongs in PyFlow.
    """
    unique_service_types.add(service_type)
    service_definitions.append(
        ServiceDefinition(service_name, service_type, pyflow_name, pyflow_group)
    )


# fmt: off
group = "Detection"
add("GetMaskProperties", "/get_mask_properties", srv.GetMaskProperties, group)
add("GetRGBDFromTopic", "/get_rgbd_from_topic", srv.GetRGBDFromTopic, group)
add("PoseFromPixel", "/pose_from_pixel", srv.PoseFromPixel, group)
add("PublishImage", "/publish_image", srv.PublishImage, group)
add("PublishMasks", "/publish_masks", srv.PublishMasks, group)
add("SegmentImage", "/segment_image", srv.SegmentImage, group)
add("SelectImageFromList", "/select_image_from_list", srv.SelectImageFromList, group)
add("SelectPickLocation", "/select_pick_location", srv.SelectPickLocation, group)
add("SelectPlaceLocation", "/select_place_location", srv.SelectPlaceLocation, group)

group = "Utilities"
add("ExpressPoseInOtherFrame", "/express_pose_in_other_frame", srv.ExpressPoseInOtherFrame, group)
add("TransformPose", "/transform_pose", srv.TransformPose, group)

group = "moveit_manager"
add("AddObject", f"{group}/add_object", srv.AddObject, group)
add("AddMarker", f"{group}/add_marker", srv.AddMarker, group)
add("DefineGoalPose", f"{group}/define_goal_pose", srv.DefineGoalPose, group)
add("ClearObjects", f"{group}/clear_objects", Trigger, group)
add("ClearMarkers", f"{group}/clear_markers", Trigger, group)
add("MoveHandToPose", f"{group}/move_hand_to_pose", srv.MoveHandToPose, group)
add("MoveToConfiguration", f"{group}/move_to_configuration", srv.MoveToConfiguration, group)

group = "Gripper"
add("OpenGripper", "/open_gripper", Trigger, group)
add("CloseGripper", "/close_gripper", Trigger, group)
