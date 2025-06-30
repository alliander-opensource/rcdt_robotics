#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import os
from dataclasses import dataclass
from pathlib import Path

import gdown
import numpy as np
import open3d as o3d
import rclpy
import torch
from graspnetAPI import GraspGroup
from graspnetpy_models.graspnet import GraspNet, pred_decode
from graspnetpy_utils import data_utils
from open3d.visualization.draw import draw
from rcdt_utilities.cv_utils import ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

CHECKPOINT_DIR = "/home/rcdt/rcdt_robotics/.cache/graspnet/"
CHECKPOINT_FILE = "checkpoint-rs.tar"
CHECKPOINT = CHECKPOINT_DIR + CHECKPOINT_FILE
DRIVE_LINK = "https://drive.google.com/uc?id=1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk"

TIMEOUT = 5.0  # seconds
NUM_VIEW = 300
NUM_POINT = 20000


@dataclass
class Message:
    """Data class to define messages that will be obtained using the wait_for_message function.

    Attributes:
        topic (str): The ROS topic to subscribe to.
        msg_type (type): The type of the message.
        ros_value (None | Image | CameraInfo): The raw ROS message received.
        value (None | np.ndarray | data_utils.CameraInfo): The processed value.
    """

    topic: str
    msg_type: type
    ros_value: None | Image | CameraInfo = None
    value: None | np.ndarray | data_utils.CameraInfo = None

    def get_message(self, node: Node) -> bool:
        """Get the message from the topic.

        Args:
            node (Node): The ROS 2 node to use.

        Returns:
            bool: True if the message was received successfully, False otherwise.
        """
        success, self.ros_value = wait_for_message(
            self.msg_type, node, self.topic, time_to_wait=TIMEOUT
        )
        if not success:
            node.get_logger().error(f"Failed to get message from {self.topic}")
        return success


class GenerateGrasp(Node):
    """Node to generate grasps using the GraspNet model."""

    def __init__(self) -> None:
        """Initialize the PublishImage node."""
        super().__init__("grasp")
        self.init_net()
        self.create_service(Trigger, "/grasp/generate", self.callback)

        self.color = Message(topic="/franka/realsense/color/image_raw", msg_type=Image)
        self.depth = Message(
            topic="/franka/realsense/depth/image_rect_raw", msg_type=Image
        )
        self.camera_info = Message(
            topic="/franka/realsense/depth/camera_info", msg_type=CameraInfo
        )

    def init_net(self) -> None:
        """Initialize the GraspNet model and load the pre-trained weights."""
        # Init the model
        self.net = GraspNet(
            input_feature_dim=0,
            num_view=NUM_VIEW,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.net.to(device)

        # Load checkpoint
        if not os.path.isfile(CHECKPOINT):
            Path(CHECKPOINT_DIR).mkdir(parents=True, exist_ok=True)
            gdown.download(url=DRIVE_LINK, output=CHECKPOINT)
        checkpoint = torch.load(CHECKPOINT)
        self.net.load_state_dict(checkpoint["model_state_dict"])

        # set model to eval mode
        self.net.eval()

    def process_data(self) -> tuple[dict, o3d.geometry.PointCloud]:
        """Process the depth and camera info data to create a point cloud and end points.

        Returns:
            tuple[dict, o3d.geometry.PointCloud]: A tuple containing the end points dictionary and the point cloud object.
        """
        cloud = data_utils.create_point_cloud_from_depth_image(
            self.depth.value, self.camera_info.value, organized=True
        )

        mask = self.depth.value > 0
        cloud_masked = cloud[mask]
        color_masked = self.color.value[mask]

        # sample points
        if len(cloud_masked) >= NUM_POINT:
            idxs = np.random.choice(len(cloud_masked), NUM_POINT, replace=False)
        else:
            idxs1 = np.arange(len(cloud_masked))
            idxs2 = np.random.choice(
                len(cloud_masked), NUM_POINT - len(cloud_masked), replace=True
            )
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]

        # convert data
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
        cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
        end_points = {}
        cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_sampled = cloud_sampled.to(device)
        end_points["point_clouds"] = cloud_sampled
        end_points["cloud_colors"] = color_sampled

        return end_points, cloud

    def get_grasps(self, end_points: dict) -> GraspGroup:
        """Define grasps from the end points using the GraspNet model.

        Args:
            end_points (dict): The end points.

        Returns:
            GraspGroup: The predicted grasp group.
        """
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        grasps_array = grasp_preds[0].detach().cpu().numpy()
        return GraspGroup(grasps_array)

    @staticmethod
    def visualize_grasps(grasps: GraspGroup, cloud: o3d.geometry.PointCloud) -> None:
        """Visualize the grasps on the point cloud.

        Args:
            grasps (GraspGroup): The predicted grasp group.
            cloud (o3d.geometry.PointCloud): The point cloud to visualize the grasps on.
        """
        grasps.nms()
        grasps.sort_by_score()
        grasps = grasps[:1]
        grippers = grasps.to_open3d_geometry_list()
        draw([cloud, *grippers])

    def get_messages(self) -> bool:
        """Get the messages from the topics and convert to correct format.

        Returns:
            bool: True if all messages are successfully retrieved, False otherwise.
        """
        for message in [self.color, self.depth, self.camera_info]:
            if not message.get_message(self):
                return False

        self.color.value = (ros_image_to_cv2_image(self.color.ros_value)) / 255.0
        self.depth.value = ros_image_to_cv2_image(self.depth.ros_value)

        self.camera_info.value = data_utils.CameraInfo(
            self.camera_info.ros_value.width,
            self.camera_info.ros_value.height,
            self.camera_info.ros_value.k[0],
            self.camera_info.ros_value.k[4],
            self.camera_info.ros_value.k[2],
            self.camera_info.ros_value.k[5],
            1000,
        )

        return True

    def callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Callback for the generate grasps service.

        Args:
            _request (Trigger.Request): The request for the service.
            response (Trigger.Response): The response to be filled.

        Returns:
            Trigger.Response: The response indicating success or failure of the grasp generation.
        """
        if not self.get_messages():
            response.success = False
            response.message = "Failed to retrieve messages."
            return response

        end_points, cloud = self.process_data()
        grasps = self.get_grasps(end_points)
        self.visualize_grasps(grasps, cloud)

        response.success = True
        response.message = "Grasps generated and visualized successfully."
        return response


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (list | None): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = GenerateGrasp()
    spin_node(node)


if __name__ == "__main__":
    main()
