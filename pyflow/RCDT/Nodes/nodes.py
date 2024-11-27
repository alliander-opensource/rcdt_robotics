# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from RCDT.Nodes.core import PyflowNode, RosService, RosNode

from sensor_msgs.msg import Image
from rclpy import wait_for_message
import cv2

from rcdt_detection.image_manipulation import cv2_image_to_ros_image

from rcdt_utilities_msgs.srv import AddMarker, AddObject, MoveRobot
from rcdt_detection_msgs.srv import SegmentImage
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
        images = self.ui.get_data("images")
        n = self.ui.input_pins["n"].getData()
        image = images[int(n)]
        self.ui.set_data("image", image)
        self.ui.success = True


class PublishImage:
    def __init__(self, ui: RosNode):
        self.ui = ui
        self.ui.input_dict = {"image": Image}
        self.ui.output_dict = {}
        self.pub = PyflowNode.node.create_publisher(Image, "test", 10)
        ui.run_async = self.run_async

    def run_async(self) -> None:
        image = self.ui.get_data("image")
        self.pub.publish(image)


class Segment:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = SegmentImage
        ui.client = PyflowNode.node.create_client(SegmentImage, "/segment_image")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = SegmentImage.Request()
        request.input_image = self.ui.get_data("input_image")
        response: SegmentImage.Response = self.ui.call_service(request)
        if response is None:
            return
        self.ui.set_data("segmented_image", response.segmented_image)
        self.ui.set_data("masks", response.masks)


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
