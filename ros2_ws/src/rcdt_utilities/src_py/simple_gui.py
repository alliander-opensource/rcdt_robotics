#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import pickle
import threading
from dataclasses import dataclass
from typing import Any

import numpy as np
import PIL.Image as PILImage
import rclpy
from geometry_msgs.msg import PoseStamped
from nicegui import app, ui
from rcdt_messages.msg import Grasp
from rcdt_messages.srv import (
    AddObject,
    GenerateGraspnetGrasp,
    MoveToConfiguration,
    PoseStampedSrv,
)
from rcdt_utilities.cv_utils import cv2_image_to_ros_image, ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Empty, SetBool, Trigger

TIMEOUT = 3


@dataclass
class Data:
    """Class to hold the data for the UI.

    Attributes:
        color_ros (Image | None): The color image in ROS format.
        color_cv (np.ndarray | None): The color image in OpenCV format.
        color_pil (PILImage.Image | None): The color image in PIL format.
        depth_ros (Image | None): The depth image in ROS format.
        depth_cv (np.ndarray | None): The depth image in OpenCV format.
        depth_pil (PILImage.Image | None): The depth image in PIL format.
        camera_info (CameraInfo | None): The camera info.
        grasp_pose (PoseStamped | None): The grasp pose.
    """

    color_ros: Image | None = None
    color_cv: np.ndarray | None = None
    color_pil: PILImage.Image | None = None
    depth_ros: Image | None = None
    depth_cv: np.ndarray | None = None
    depth_pil: PILImage.Image | None = None
    camera_info: CameraInfo | None = None

    grasp_pose: PoseStamped | None = None

    def update(self) -> None:
        """Update the OpenCV and PIL images from the ROS images."""
        if self.color_ros is not None:
            self.color_cv = ros_image_to_cv2_image(self.color_ros)
            self.color_pil = PILImage.fromarray(self.color_cv)
        if self.depth_ros is not None:
            self.depth_cv = ros_image_to_cv2_image(self.depth_ros)
            self.depth_pil = PILImage.fromarray(self.depth_cv)


@dataclass
class Message:
    """Data class to define messages that will be obtained using the wait_for_message function.

    Attributes:
        topic (str): The ROS topic to subscribe to.
        message_type (type): The type of the message.
    """

    topic: str
    message_type: type

    def get_message(self, node: Node) -> Any:
        """Get the message from the topic.

        Args:
            node (Node): The ROS 2 node to use.

        Returns:
            Any: The received message.
        """
        success, message = wait_for_message(
            self.message_type, node, self.topic, time_to_wait=TIMEOUT
        )
        if not success:
            node.get_logger().error(f"Failed to get message from {self.topic}")
        return message


class UI(Node):
    """Node of the UI."""

    def __init__(self):
        """Initialize the node."""
        super().__init__("simple_gui")

        self.image_placeholder = np.ones((480, 640), dtype=np.uint8) * 150
        self.data = Data()

        self.color_image: ui.image
        self.depth_image: ui.image
        self.camera_info: ui.label
        self.grasp_info: ui.label

        self.clear_objects_client = self.create_client(
            Trigger, "/franka1/moveit_manager/clear_objects"
        )

        self.add_object_client = self.create_client(
            AddObject, "/franka1/moveit_manager/add_object"
        )

        self.toggle_octomap_scan_client = self.create_client(
            SetBool, "/franka1/moveit_manager/toggle_octomap_scan"
        )

        self.clear_octomap_client = self.create_client(Empty, "/franka1/clear_octomap")

        self.move_to_configuration_client = self.create_client(
            MoveToConfiguration, "/franka1/moveit_manager/move_to_configuration"
        )

        self.generate_grasp_client = self.create_client(
            GenerateGraspnetGrasp, "/graspnet/generate"
        )

        self.visualize_grasp_pose_client = self.create_client(
            PoseStampedSrv, "/franka1/moveit_manager/visualize_grasp_pose"
        )

        self.create_plan_client = self.create_client(
            PoseStampedSrv, "/franka1/moveit_manager/create_plan"
        )

        self.visualize_plan_client = self.create_client(
            Trigger, "/franka1/moveit_manager/visualize_plan"
        )

        self.execute_plan_client = self.create_client(
            Trigger, "/franka1/moveit_manager/execute_plan"
        )

        self.setup_gui()

    def setup_gui(self) -> None:
        """Setup the GUI pages."""

        @ui.page("/")
        def page() -> None:
            """Setup the page of the GUI."""
            with ui.row():
                ui.button("Load Data", on_click=self.load)
                ui.button("Save Data", on_click=self.save)
                ui.button("Move Home", on_click=self.move_to_home)
            with ui.row():
                ui.button("Add Object", on_click=self.add_object)
                ui.button("Clear Objects", on_click=self.clear_objects)
                ui.button(
                    "Enable Octomap Scan",
                    on_click=lambda: self.toggle_octomap_scan(True),
                )
                ui.button(
                    "Disable Octomap Scan",
                    on_click=lambda: self.toggle_octomap_scan(False),
                )
                ui.button("Clear Octomap", on_click=self.clear_octomap)
            with ui.row():
                self.color_image = ui.image(self.data.color_pil).classes("w-32")
                self.depth_image = ui.image(self.data.depth_pil).classes("w-32")
                self.camera_info = ui.label("No camera info.")
            with ui.row():
                ui.button("Update Image", on_click=lambda: self.update_image("color"))
                ui.button("Update Image", on_click=lambda: self.update_image("depth"))
                ui.button("Update Camera Info", on_click=self.update_camera_info)
            self.grasp_info = ui.label("No grasp generated.")
            with ui.row():
                ui.button("Generate Grasp", on_click=self.generate_grasp)
                ui.button("Visualize Grasp", on_click=self.visualize_grasp_pose)
                ui.button("Create Plan", on_click=self.create_plan)
                ui.button("Visualize Plan", on_click=self.visualize_plan)
                ui.button("Execute Plan", on_click=self.execute_plan)

    def update_ui(self) -> None:
        """Update the UI."""
        self.color_image.source = self.data.color_pil
        self.depth_image.source = self.data.depth_pil
        self.camera_info.text = (
            self.data.camera_info.header.stamp.sec
            if self.data.camera_info
            else "No camera info."
        )
        self.grasp_info.text = (
            str(self.data.grasp_pose.pose.position)
            if self.data.grasp_pose
            else "No grasp generated."
        )

    def load(self) -> None:
        """Load the data from a file."""
        try:
            with open("/tmp/data.pkl", "rb") as f:
                self.data = pickle.load(f)
        except FileNotFoundError:
            self.get_logger().error("No saved data found.")
        self.update_ui()

    def save(self) -> None:
        """Save the data to a file."""
        with open("/tmp/data.pkl", "wb") as f:
            pickle.dump(self.data, f)

    def move_to_home(self) -> None:
        """Move the robot to the home configuration."""
        request = MoveToConfiguration.Request()
        request.configuration = "home"
        if self.move_to_configuration_client.call(request, TIMEOUT) is None:
            self.get_logger().error("Failed to call move to configuration service.")
        else:
            self.get_logger().info("Successfully called move to configuration service.")

    def execute_plan(self) -> None:
        """Execute the plan."""
        if self.execute_plan_client.call(Trigger.Request(), TIMEOUT) is None:
            self.get_logger().error("Failed to call execute plan service.")
        else:
            self.get_logger().info("Successfully called execute plan service.")

    def visualize_plan(self) -> None:
        """Visualize the plan in Rviz."""
        if self.visualize_plan_client.call(Trigger.Request(), TIMEOUT) is None:
            self.get_logger().error("Failed to call visualize plan service.")
        else:
            self.get_logger().info("Successfully called visualize plan service.")

    def create_plan(self) -> None:
        """Create a plan to reach the grasp pose."""
        request = PoseStampedSrv.Request()
        request.pose = self.data.grasp_pose
        if self.create_plan_client.call(request, TIMEOUT) is None:
            self.get_logger().error("Failed to call create plan service.")
        else:
            self.get_logger().info("Successfully called create plan service.")

    def visualize_grasp_pose(self) -> None:
        """Visualize the grasp pose in Rviz."""
        if self.data.grasp_pose is None:
            self.get_logger().error("No grasp pose to visualize.")
            return
        request = PoseStampedSrv.Request()
        request.pose = self.data.grasp_pose
        if self.visualize_grasp_pose_client.call(request, TIMEOUT) is None:
            self.get_logger().error("Failed to call visualize gripper pose service.")
        else:
            self.get_logger().info(
                "Successfully called visualize gripper pose service."
            )

    def clear_objects(self) -> None:
        """Clear objects from the planning scene."""
        if self.clear_objects_client.call(Trigger.Request(), TIMEOUT) is None:
            self.get_logger().error("Failed to call clear objects service.")
        else:
            self.get_logger().info("Successfully called clear objects service.")

    def toggle_octomap_scan(self, enable: bool) -> None:
        """Enable or disable octomap scanning.

        Args:
            enable (bool): True to enable scanning, False to disable.
        """
        request = SetBool.Request()
        request.data = enable

        if self.toggle_octomap_scan_client.call(request, TIMEOUT) is None:
            self.get_logger().error("Failed to call toggle octomap scan service.")
        else:
            self.get_logger().info("Successfully called toggle octomap scan service.")

    def clear_octomap(self) -> None:
        """Clear the octomap."""
        if self.clear_octomap_client.call(Empty.Request(), TIMEOUT) is None:
            self.get_logger().error("Failed to call clear octomap service.")
        else:
            self.get_logger().info("Successfully called clear octomap service.")

    def add_object(self) -> None:
        """Add an object to the planning scene."""
        request = AddObject.Request()
        request.shape = "SPHERE"
        request.pose.header.frame_id = "franka1/fr3_link0"
        request.d1 = 0.1  # radius

        if self.add_object_client.call(request, TIMEOUT) is None:
            self.get_logger().error("Failed to call add object service.")
        else:
            self.get_logger().info("Successfully called add object service.")

    def update_image(self, image_type: str) -> None:
        """Update the image from the ROS topic.

        Args:
            image_type (str): The type of image to update ("color" or "depth").
        """
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
        setattr(self.data, f"{image_type}_ros", image_ros)
        self.data.update()
        self.update_ui()

    def update_camera_info(self) -> None:
        """Update the camera info from the ROS topic."""
        camera_info: CameraInfo = Message(
            topic="/realsense1/depth/camera_info", message_type=CameraInfo
        ).get_message(self)
        if camera_info:
            self.data.camera_info = camera_info
            self.update_ui()

    def generate_grasp(self) -> None:
        """Generate a grasp using the Graspnet service."""
        request = GenerateGraspnetGrasp.Request()
        request.color = self.data.color_ros
        request.depth = cv2_image_to_ros_image(self.data.depth_cv / 1000)
        request.depth.header = self.data.depth_ros.header
        request.camera_info = self.data.camera_info

        if not self.generate_grasp_client.wait_for_service(TIMEOUT):
            self.get_logger().error("Generate Graspnet Grasp service not available.")
            return
        response: GenerateGraspnetGrasp.Response = self.generate_grasp_client.call(
            request, 10
        )
        grasp: Grasp = response.grasps[0]
        self.data.grasp_pose = grasp.pose
        self.update_ui()


def ros_main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and start the executor.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    node = UI()
    spin_node(node)


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui.run()
