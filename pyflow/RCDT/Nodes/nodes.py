# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from RCDT.Nodes.core import PyflowNode, RosService, RosNode

from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import RGBD
from rclpy import wait_for_message
import cv2

from rcdt_utilities.cv_utils import cv2_image_to_ros_image

from rcdt_utilities_msgs.srv import (
    AddMarker,
    AddObject,
    MoveToPose,
    MoveToConfiguration,
    TransformPose,
    ExpressPoseInOtherFrame,
)
from rcdt_detection_msgs.srv import (
    SegmentImage,
    FilterMasks,
    DefineCentroid,
    PoseFromPixel,
    SplitRGBD,
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
        request.marker_pose = self.ui.get_data("marker_pose")
        self.ui.call_service(request)


class TransformPoseNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = TransformPose
        ui.client = PyflowNode.node.create_client(
            TransformPose, "/manipulate_pose/transform_pose_relative"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = TransformPose.Request()
        request.pose = self.ui.get_data("pose")
        request.transform = self.ui.get_data("transform")
        response: TransformPose.Response = self.ui.call_service(request)
        self.ui.set_pins_based_on_response(response)


class ExpressPoseInOtherFrameNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = ExpressPoseInOtherFrame
        ui.client = PyflowNode.node.create_client(
            ExpressPoseInOtherFrame, "/manipulate_pose/express_pose_in_other_frame"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = ExpressPoseInOtherFrame.Request()
        request.pose = self.ui.get_data("pose")
        request.target_frame = self.ui.get_data("target_frame")
        response: ExpressPoseInOtherFrame.Response = self.ui.call_service(request)
        self.ui.set_pins_based_on_response(response)


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
            topic=self.ui.get_data("topic"),
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
            topic=self.ui.get_data("topic"),
            time_to_wait=1,
        )
        if success:
            self.ui.set_data("image", message)
            self.ui.success = True


class GetRGBDFromTopic:
    def __init__(self, ui: RosNode):
        self.ui = ui
        self.ui.input_dict = {"topic": str}
        self.ui.output_dict = {"rgbd": RGBD}
        ui.run_async = self.run_async

    def run_async(self) -> None:
        success, message = wait_for_message.wait_for_message(
            msg_type=RGBD,
            node=PyflowNode.node,
            topic=self.ui.get_data("topic"),
            time_to_wait=1,
        )
        if success:
            self.ui.set_data("rgbd", message)
            self.ui.success = True


class GetImageFromFile:
    def __init__(self, ui: RosNode):
        self.ui = ui
        self.ui.input_dict = {"path": str}
        self.ui.output_dict = {"image": Image}
        ui.run_async = self.run_async

    def run_async(self) -> None:
        cv2_image = cv2.imread(self.ui.get_data("path"))
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
        n = self.ui.get_data("n")
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
        image = self.ui.get_data("image")
        if not image:
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
        request.input_image = self.ui.get_data("input_image")
        response: SegmentImage.Response = self.ui.call_service(request)
        self.ui.set_pins_based_on_response(response)


class SplitRGBDNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = SplitRGBD
        ui.client = PyflowNode.node.create_client(SplitRGBD, "/split_rgbd")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = SplitRGBD.Request()
        request.rgbd = self.ui.get_data("rgbd")
        response: SplitRGBD.Response = self.ui.call_service(request)
        self.ui.set_pins_based_on_response(response)


class Filter:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = FilterMasks
        ui.client = PyflowNode.node.create_client(FilterMasks, "/filter_masks")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = FilterMasks.Request()
        request.masks = self.ui.get_data("masks")
        request.filter_method = "brick"
        response: FilterMasks.Response = self.ui.call_service(request)
        self.ui.set_pins_based_on_response(response)


class DefineCentroidNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = DefineCentroid
        ui.client = PyflowNode.node.create_client(DefineCentroid, "/define_centroid")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = DefineCentroid.Request()
        request.image = self.ui.get_data("image")
        response: DefineCentroid.Response = self.ui.call_service(request)
        self.ui.set_pins_based_on_response(response)


class PoseFromPixelNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = PoseFromPixel
        ui.client = PyflowNode.node.create_client(PoseFromPixel, "/pose_from_pixel")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = PoseFromPixel.Request()
        request.pixel = self.ui.get_data("pixel")
        request.depth_image = self.ui.get_data("depth_image")
        request.camera_info = self.ui.get_data("camera_info")
        response: PoseFromPixel.Response = self.ui.call_service(request)
        self.ui.set_pins_based_on_response(response)


class MoveHandToPoseNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = MoveToPose
        ui.client = PyflowNode.node.create_client(
            MoveToPose, "/moveit_controller/move_hand_to_pose"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = MoveToPose.Request()
        request.pose = self.ui.get_data("pose")
        request.pose.header.frame_id = "fr3_link0"
        self.ui.call_service(request)


class MoveToConfigurationNode:
    def __init__(self, ui: RosService):
        self.ui = ui
        ui.service = MoveToConfiguration
        ui.client = PyflowNode.node.create_client(
            MoveToConfiguration, "/moveit_controller/move_to_configuration"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = MoveToConfiguration.Request()
        request.configuration = self.ui.get_data("configuration")
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
        request.object = self.ui.get_data("object")
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
