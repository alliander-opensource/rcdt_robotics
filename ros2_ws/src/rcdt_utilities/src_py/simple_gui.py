#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import threading
from dataclasses import dataclass

import numpy as np
import rclpy
from nicegui import app, ui
from PIL import Image as PIL
from rcdt_messages.msg import Grasp
from rcdt_messages.srv import GenerateGraspnetGrasp
from rcdt_utilities.cv_utils import cv2_image_to_ros_image, ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CameraInfo, Image

TIMEOUT = 3


@dataclass
class CameraView:
    color_ros: Image | None = None
    color_cv: np.ndarray | None = None
    color: PIL.Image | None = None
    depth_ros: Image | None = None
    depth_cv: np.ndarray | None = None
    depth: PIL.Image | None = None
    camera_info: CameraInfo | None = None

    def update(self) -> None:
        if self.color_ros is not None:
            self.color_cv = ros_image_to_cv2_image(self.color_ros)
            self.color = PIL.fromarray(self.color_cv)
        if self.depth_ros is not None:
            self.depth_cv = ros_image_to_cv2_image(self.depth_ros)
            self.depth = PIL.fromarray(self.depth_cv)


@dataclass
class Message:
    """Data class to define messages that will be obtained using the wait_for_message function.

    Attributes:
        topic (str): The ROS topic to subscribe to.
        message_type (type): The type of the message.
    """

    topic: str
    message_type: type

    def get_message(self, node: Node) -> object:
        """Get the message from the topic.

        Args:
            node (Node): The ROS 2 node to use.

        Returns:
            object: The received message.
        """
        success, message = wait_for_message(
            self.message_type, node, self.topic, time_to_wait=TIMEOUT
        )
        if not success:
            node.get_logger().error(f"Failed to get message from {self.topic}")
        return message


class ManipulatePose(Node):
    """Node to manipulate poses in different frames and apply transformations."""

    def __init__(self):
        """Initialize the node."""
        super().__init__("simple_gui")

        self.image_placeholder = np.ones((480, 640), dtype=np.uint8) * 150
        self.camera_view = CameraView()

        self.generate_grasp_client = self.create_client(
            GenerateGraspnetGrasp, "/graspnet/generate"
        )

        self.setup_gui()

    def setup_gui(self) -> None:
        """Setup the GUI pages."""

        @ui.page("/")
        def page():
            with ui.row():
                color_image = ui.image(self.camera_view.color).classes("w-32")
                depth_image = ui.image(self.camera_view.depth).classes("w-32")
                camera_info = ui.label("No camera info.").classes("m-2")
            with ui.row():
                ui.button(
                    "Update Image",
                    on_click=lambda: self.update_image(color_image, "color"),
                )
                ui.button(
                    "Update Image",
                    on_click=lambda: self.update_image(depth_image, "depth"),
                )
                ui.button(
                    "Update Camera Info",
                    on_click=lambda: self.update_camera_info(camera_info),
                )
            grasp = ui.label("No grasp generated.").classes("m-2")
            ui.button("Generate Grasp", on_click=lambda: self.generate_grasp(grasp))

    def update_image(self, ui: ui.image, image_type: str) -> None:
        match image_type:
            case "color":
                topic = "/realsense1/color/image_raw"
            case "depth":
                topic = "/realsense1/depth/image_rect_raw"
            case _:
                return

        image_ros: Image = Message(topic=topic, message_type=Image).get_message(self)
        if image_ros is None:
            return
        setattr(self.camera_view, f"{image_type}_ros", image_ros)
        self.camera_view.update()
        ui.source = getattr(self.camera_view, image_type)

    def update_camera_info(self, ui: ui.label) -> None:
        camera_info: CameraInfo = Message(
            topic="/realsense1/depth/camera_info", message_type=CameraInfo
        ).get_message(self)
        if camera_info:
            self.camera_view.camera_info = camera_info
            ui.text = str(camera_info.header.stamp.sec)

    def generate_grasp(self, ui: ui.label) -> None:
        request = GenerateGraspnetGrasp.Request()
        request.color = self.camera_view.color_ros
        request.depth = cv2_image_to_ros_image(self.camera_view.depth_cv / 1000)
        request.camera_info = self.camera_view.camera_info

        if not self.generate_grasp_client.wait_for_service(TIMEOUT):
            self.get_logger().error("Generate Graspnet Grasp service not available.")
            return
        response: GenerateGraspnetGrasp.Response = self.generate_grasp_client.call(
            request, 10
        )
        grasp: Grasp = response.grasps[0]
        ui.text = str(grasp.pose.pose.position)


def ros_main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and start the executor.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    node = ManipulatePose()
    spin_node(node)


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui.run()
