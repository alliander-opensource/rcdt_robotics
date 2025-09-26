#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import numpy as np
import open3d as o3d
import rclpy
import torch
from graspnetAPI import GraspGroup
from graspnetpy_models.graspnet import GraspNet, pred_decode
from graspnetpy_utils import collision_detector, data_utils
from rcdt_messages.msg import Grasp
from rcdt_messages.srv import GenerateGraspnetGrasp
from rcdt_utilities.cv_utils import ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node
from scipy.spatial.transform import Rotation

ros_logger = logging.get_logger(__name__)

CHECKPOINT_DIR = "/home/rcdt/rcdt_robotics/ros2_ws/src/rcdt_grasping/checkpoint/"
CHECKPOINT_FILE = "checkpoint.tar"
CHECKPOINT = CHECKPOINT_DIR + CHECKPOINT_FILE

NUM_VIEW = 300
NUM_POINT = 20000


class GenerateGrasp(Node):
    """Node to generate grasps using the GraspNet model."""

    def __init__(self) -> None:
        """Initialize the PublishImage node."""
        super().__init__("graspnet_node")
        self.init_net()

        self.collision_threshold = 0.01
        self.voxel_size = 0.01
        self.visualize = False

        self.depth = None
        self.color = None
        self.camera_info = None
        self.depth_frame_id = None

        self.create_service(GenerateGraspnetGrasp, "/graspnet/generate", self.callback)

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

        checkpoint = torch.load(CHECKPOINT)
        self.net.load_state_dict(checkpoint["model_state_dict"])

        # set model to eval mode
        self.net.eval()

        ros_logger.info("GraspNet model initialized and weights loaded")

    def callback(
        self,
        request: GenerateGraspnetGrasp.Request,
        response: GenerateGraspnetGrasp.Response,
    ) -> GenerateGraspnetGrasp.Response:
        """Callback for the generate grasps service.

        Args:
            request (GenerateGraspnetGrasp.Request): The request for the service.
            response (GenerateGraspnetGrasp.Response): The response to be filled.

        Returns:
            GenerateGraspnetGrasp.Response: The response indicating success or failure of the grasp generation.
        """
        if not self.convert_messages(request):
            response.success = False
            response.message = "Failed to convert messages"
            return response

        # Define cloud and grasps:
        end_points, cloud = self.process_data()

        grasps = self.get_grasps(end_points)

        if len(grasps) == 0:
            response.success = False
            response.message = "GraspNet returned 0 grasps"
            self.get_logger().warn("GraspNet returned 0 grasps")
            return response

        ros_logger.info(f"GraspNet returned {len(grasps)} grasps")

        grasps.nms()
        grasps.sort_by_score()
        grasps = self.collision_detection(grasps, np.array(cloud.points))
        if self.visualize:
            self.vis_grasps(grasps, cloud)
        grasps_msg = self.grasps_to_ros_msg(grasps)
        response.success = True
        response.message = "worked!"
        response.grasps = grasps_msg
        ros_logger.info("Returning success response!")
        return response

    def grasps_to_ros_msg(self, grasps: GraspGroup) -> list[Grasp]:
        """Convert the GraspGroup to a list of ROS Grasp messages.

        Args:
            grasps (GraspGroup): The GraspGroup containing the grasps to be converted.

        Returns:
            list[Grasp]: A list of ROS Grasp messages.
        """
        msgs: list[Grasp] = []
        for g in grasps:
            t = np.asarray(g.translation, dtype=float).reshape(3)
            rotation_matrix = np.asarray(
                getattr(g, "rotation", g.rotation_matrix), dtype=float
            ).reshape(3, 3)

            rotation_orig = Rotation.from_matrix(rotation_matrix)
            qx, qy, qz, qw = rotation_orig.as_quat()

            m = Grasp()
            m.pose.header.frame_id = self.depth_frame_id
            m.pose.pose.position.x = float(t[0])
            m.pose.pose.position.y = float(t[1])
            m.pose.pose.position.z = float(t[2])
            m.pose.pose.orientation.x = float(qx)
            m.pose.pose.orientation.y = float(qy)
            m.pose.pose.orientation.z = float(qz)
            m.pose.pose.orientation.w = float(qw)

            m.score = float(g.score)
            m.width = float(g.width)
            m.depth = float(g.depth)

            msgs.append(m)
        return msgs

    def process_data(self) -> tuple[dict, o3d.geometry.PointCloud]:  # type: ignore[attr-defined]
        """Process the depth and camera info data to create a point cloud and end points.

        Returns:
            tuple[dict, o3d.geometry.PointCloud]: A tuple containing the end points dictionary and the point cloud object.
        """
        cloud = data_utils.create_point_cloud_from_depth_image(
            self.depth, self.camera_info, organized=True
        )

        mask = self.depth > 0
        cloud_masked = cloud[mask]
        color_masked = self.color[mask]

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
        cloud = o3d.geometry.PointCloud()  # type: ignore[attr-defined]
        cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))  # type: ignore[attr-defined]
        cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))  # type: ignore[attr-defined]
        end_points = {}
        cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_sampled = cloud_sampled.to(device)
        end_points["point_clouds"] = cloud_sampled
        end_points["cloud_colors"] = color_sampled

        assert len(cloud.points) > 0, "PointCloud has no points"
        ros_logger.info(f"PointCloud has {len(cloud.points)} points")
        return end_points, cloud

    def get_grasps(self, end_points: dict) -> GraspGroup:
        """Define grasps from the end points using the GraspNet model.

        Args:
            end_points (dict): The end points dictionary containing the point cloud and colors.

        Returns:
            GraspGroup: The GraspGroup containing the predicted grasps.
        """
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)

        grasps_array = grasp_preds[0].detach().cpu().numpy()
        return GraspGroup(grasps_array)

    def collision_detection(self, grasps: GraspGroup, cloud: np.ndarray) -> GraspGroup:
        """Perform collision detection on the grasps using the point cloud.

        Args:
            grasps (GraspGroup): The GraspGroup containing the grasps to be checked.
            cloud (np.ndarray): The point cloud as a numpy array.

        Returns:
            GraspGroup: The filtered GraspGroup after collision detection.
        """
        mfcdetector = collision_detector.ModelFreeCollisionDetector(
            cloud, voxel_size=self.voxel_size
        )
        collision_mask = mfcdetector.detect(
            grasps, approach_dist=0.05, collision_thresh=self.collision_threshold
        )
        grasps = grasps[~collision_mask]
        ros_logger.info(
            f"Collision detection filtered {np.sum(collision_mask)} grasps, {len(grasps)} grasps remaining"
        )
        return grasps

    @staticmethod
    def vis_grasps(gg: GraspGroup, cloud: np.ndarray) -> None:
        """Visualize the grasps using Open3D.

        Args:
            gg (GraspGroup): The GraspGroup containing the grasps to visualize.
            cloud (np.ndarray): The point cloud as a numpy array.
        """
        gg = gg[:1]
        grippers = gg.to_open3d_geometry_list()
        ros_logger.info(f"Visualizing {len(grippers)} grasps")

        o3d.visualization.draw_geometries([cloud, *grippers])  # type: ignore[attr-defined]

    def convert_messages(self, request: GenerateGraspnetGrasp.Request) -> bool:
        """Convert the messages to correct format.

        Args:
            request (GenerateGraspnetGrasp.Request): The request containing the messages.

        Returns:
            bool: True if all messages are successfully retrieved, False otherwise.
        """
        try:
            self.color = (ros_image_to_cv2_image(request.color)) / 255.0
            self.depth = ros_image_to_cv2_image(request.depth)
            self.depth_frame_id = request.depth.header.frame_id
            # Camera intrinsics
            self.camera_info = data_utils.CameraInfo(
                request.camera_info.width,
                request.camera_info.height,
                request.camera_info.k[0],
                request.camera_info.k[4],
                request.camera_info.k[2],
                request.camera_info.k[5],
                1,
            )
            return True
        except Exception as e:
            ros_logger.error(f"Error converting messages: {e}")
            return False


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
