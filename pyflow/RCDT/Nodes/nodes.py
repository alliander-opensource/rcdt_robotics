# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from RCDT.Nodes.core import PyflowNode, RosService, RosNode

from sensor_msgs.msg import Image, CameraInfo
from rclpy import wait_for_message
import cv2

from rcdt_detection.image_manipulation import cv2_image_to_ros_image

from rcdt_utilities_msgs.srv import AddMarker, AddObject, MoveRobot
from rcdt_detection_msgs.srv import (
    SegmentImage,
    FilterMasks,
    DefineCentroid,
    PointFromPixel,
)
from std_srvs.srv import Trigger


class RvizMark:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = AddMarker
        ui.client = PyflowNode.node.create_client(
            AddMarker, "/rviz_controller/add_marker"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = AddMarker.Request()
        request.marker_pose = self.ui.get_message("marker_pose")
        self.ui.call_service(request)


class GetCameraInfo:
    def __init__(self, ui: RosNode):
        self.ui = ui
        self.ui.input_dict = {"topic": str}
        self.ui.output_dict = {"camera_info": CameraInfo}
        ui.run_async = self.run_async

    def run_async(self) -> None:
        success, message = wait_for_message.wait_for_message(
            msg_type=CameraInfo,
            node=PyflowNode.node,
            topic=self.ui.get_string("topic"),
            time_to_wait=1,
        )
        if success:
            self.ui.set_data("camera_info", message)
            self.ui.success = True


class GetImageFromTopic:
    def __init__(self, ui: RosNode):
        self.ui = ui
        self.ui.input_dict = {"topic": str}
        self.ui.output_dict = {"image": Image}
        ui.run_async = self.run_async

    def run_async(self) -> None:
        success, message = wait_for_message.wait_for_message(
            msg_type=Image,
            node=PyflowNode.node,
            topic=self.ui.get_string("topic"),
            time_to_wait=1,
        )
        if success:
            self.ui.set_data("image", message)
            self.ui.success = True


class GetImageFromFile:
    def __init__(self, ui: RosNode):
        self.ui = ui
        self.ui.input_dict = {"path": str}
        self.ui.output_dict = {"image": Image}
        ui.run_async = self.run_async

    def run_async(self) -> None:
        pin = self.ui.input_pins["path"]
        cv2_image = cv2.imread(pin.getData())
        ros_image = cv2_image_to_ros_image(cv2_image, "rgb8")
        self.ui.set_data("image", ros_image)
        self.ui.success = True


class GetImageFromList:
    def __init__(self, ui: RosNode):
        self.ui = ui
        self.ui.input_dict = {"images": list[Image], "n": str}
        self.ui.output_dict = {"image": Image}
        ui.run_async = self.run_async

    def run_async(self) -> None:
        succes, images = self.ui.get_data("images")
        if not succes:
            return
        n = self.ui.input_pins["n"].getData()
        image = images[int(n)]
        self.ui.set_data("image", image)
        self.ui.success = True


class PublishImage:
    def __init__(self, ui: RosNode):
        self.ui = ui
        self.ui.input_dict = {"image": Image}
        self.ui.output_dict = {}
        self.pub = PyflowNode.node.create_publisher(Image, "/debug_image", 10)
        ui.run_async = self.run_async

    def run_async(self) -> None:
        succes, image = self.ui.get_data("image")
        if not succes:
            return
        self.pub.publish(image)


class Segment:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = SegmentImage
        ui.client = PyflowNode.node.create_client(SegmentImage, "/segment_image")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = SegmentImage.Request()
        succes, input_image = self.ui.get_data("input_image")
        if not succes:
            return
        request.input_image = input_image
        response: SegmentImage.Response = self.ui.call_service(request)
        if response is None:
            return
        self.ui.set_data("segmented_image", response.segmented_image)
        self.ui.set_data("masks", response.masks)


class Filter:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = FilterMasks
        ui.client = PyflowNode.node.create_client(FilterMasks, "/filter_masks")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = FilterMasks.Request()
        success, masks = self.ui.get_data("masks")
        if not success:
            return
        request.masks = masks
        request.filter_method = "brick"
        response: FilterMasks.Response = self.ui.call_service(request)
        if response is None:
            return
        self.ui.set_data("masks", response.masks)


class DefineCentroidNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = DefineCentroid
        ui.client = PyflowNode.node.create_client(DefineCentroid, "/define_centroid")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = DefineCentroid.Request()
        success, image = self.ui.get_data("image")
        if not success:
            return
        request.image = image
        response: DefineCentroid.Response = self.ui.call_service(request)
        if response is None:
            return
        self.ui.set_data("image", response.image)


class PointFromPixelNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = PointFromPixel
        ui.client = PyflowNode.node.create_client(PointFromPixel, "/point_from_pixel")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = PointFromPixel.Request()
        success, depth_image = self.ui.get_data("depth_image")
        if not success:
            return
        success, camera_info = self.ui.get_data("camera_info")
        if not success:
            return
        request.pixel = self.ui.get_message("pixel")
        request.depth_image = depth_image
        request.camera_info = camera_info
        response: PointFromPixel.Response = self.ui.call_service(request)
        if response is None:
            return
        self.ui.set_data("point", response.point)


class MoveitMoveRobot:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = MoveRobot
        ui.client = PyflowNode.node.create_client(
            MoveRobot, "/moveit_controller/move_robot"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = MoveRobot.Request()
        request.goal_pose = self.ui.get_message("goal_pose")
        request.goal_pose.header.frame_id = "fr3_link0"
        self.ui.call_service(request)


class MoveitAddObject:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = AddObject
        ui.client = PyflowNode.node.create_client(
            AddObject, "/moveit_controller/add_object"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = AddObject.Request()
        request.object = self.ui.get_message("object")
        self.ui.call_service(request)


class MoveitClearObjects:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = Trigger
        ui.client = PyflowNode.node.create_client(
            Trigger, "/moveit_controller/clear_objects"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = Trigger.Request()
        self.ui.call_service(request)
