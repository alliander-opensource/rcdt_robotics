#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import numpy as np
import rclpy
import ros2_numpy as rnp
from easy_handeye2_msgs.srv import ComputeCalibration, TakeSample
from geometry_msgs.msg import PoseStamped, TransformStamped
from rcdt_messages.srv import (
    AddMarker,
    AddObject,
    DefineGoalPose,
    ExpressPoseInOtherFrame,
    MoveHandToPose,
    MoveJoint,
    MoveToConfiguration,
    TransformPose,
)
from rcdt_utilities.geometry import Point3D, Quaternion
from rcdt_utilities.launch_utils import spin_executor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

LAST_JOINT = 6
LAST_JOINT_DEFAULT = 45
LAST_JOINT_OFFSET = 45

DIST_MARKER = 0.6
DIST_SCAN = 0.25
ROUND = 360
GROUND_ANGLE = 45

STEPS = 5
ROUNDS = 3


class Calibrate(Node):
    def __init__(self) -> None:
        super().__init__("calibrate")
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        client_group = MutuallyExclusiveCallbackGroup()
        self.add_marker = self.create_client(
            AddMarker,
            "moveit_manager/add_marker",
            callback_group=client_group,
        )
        self.clear_markers_client = self.create_client(
            Trigger,
            "moveit_manager/clear_markers",
            callback_group=client_group,
        )
        self.add_object = self.create_client(
            AddObject, "moveit_manager/add_object", callback_group=client_group
        )
        self.define_goal_pose = self.create_client(
            DefineGoalPose,
            "moveit_manager/define_goal_pose",
            callback_group=client_group,
        )
        self.move_hand_to_pose = self.create_client(
            MoveHandToPose,
            "moveit_manager/move_hand_to_pose",
            callback_group=client_group,
        )
        self.move_to_configuration = self.create_client(
            MoveToConfiguration,
            "moveit_manager/move_to_configuration",
            callback_group=client_group,
        )
        self.move_joint = self.create_client(
            MoveJoint,
            "moveit_manager/move_joint",
            callback_group=client_group,
        )
        self.transform_pose = self.create_client(
            TransformPose,
            "/transform_pose",
            callback_group=client_group,
        )
        self.express_pose_in_other_frame = self.create_client(
            ExpressPoseInOtherFrame,
            "/express_pose_in_other_frame",
            callback_group=client_group,
        )
        self.take_sample = self.create_client(
            TakeSample,
            "/easy_handeye2/calibration/take_sample",
            callback_group=client_group,
        )
        self.compute_calibration = self.create_client(
            ComputeCalibration,
            "/easy_handeye2/calibration/compute_calibration",
            callback_group=client_group,
        )

        self.create_service(Trigger, "calibrate", self.calibrate)

    def calibrate(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.clear_markers()
        self.move_to_calibration_start()
        pose = self.estimate_marker_pose()
        self.add_ground_limit(pose)
        self.publish_origin(pose)
        poses = self.get_calibration_poses()
        for _ in range(ROUNDS):
            self.clear_markers()
            self.execution_round(poses)
        self.move_to_calibration_start()
        self.compute_transform()
        response.success = True
        return response

    def clear_markers(self) -> None:
        self.clear_markers_client.call(Trigger.Request())

    def move_to_calibration_start(self) -> None:
        self.move_to_configuration.call(
            MoveToConfiguration.Request(configuration="calibrate")
        )

    def rotate_joint(self, joint: int, deg_value: int) -> None:
        request = MoveJoint.Request(joint=joint, value=float(deg_value))
        self.move_joint.call(request)

    def estimate_marker_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "fr3_hand_tcp"
        pose.pose.position = Point3D(0.0, 0.0, DIST_MARKER).as_ros_point
        return pose

    def add_ground_limit(self, pose: PoseStamped) -> None:
        self.add_object.call(
            AddObject.Request(pose=pose, shape="BOX", d1=0.5, d2=0.5, d3=0.01)
        )

    def publish_origin(self, pose: PoseStamped) -> None:
        request = ExpressPoseInOtherFrame.Request(target_frame="world", pose=pose)
        response: ExpressPoseInOtherFrame.Response = (
            self.express_pose_in_other_frame.call(request)
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "calibration_origin"
        t.transform.translation.x = response.pose.pose.position.x
        t.transform.translation.y = response.pose.pose.position.y
        t.transform.translation.z = response.pose.pose.position.z
        t.transform.rotation = response.pose.pose.orientation
        self.tf_static_broadcaster.sendTransform(t)

    def get_calibration_poses(self) -> list[PoseStamped]:
        request = TransformPose.Request()
        request.pose.header.frame_id = "calibration_origin"
        request.pose.pose.position = Point3D(0.0, 0.0, -DIST_SCAN).as_ros_point

        poses = []
        for n in np.linspace(0, ROUND, STEPS + 1):
            rotation = Quaternion.from_eulerangles_deg(
                0, -GROUND_ANGLE, n
            ).as_ros_quaternion
            request.transform.rotation = rotation
            response: TransformPose.Response = self.transform_pose.call(request)
            poses.append(response.pose)
        poses.pop()
        return poses

    def take_calibration_sample(self) -> None:
        self.take_sample.call(TakeSample.Request())

    def compute_transform(self) -> None:
        response: ComputeCalibration.Response = self.compute_calibration.call(
            ComputeCalibration.Request()
        )
        translation = rnp.numpify(response.calibration.transform.translation)
        rotation = rnp.numpify(response.calibration.transform.rotation)

        self.get_logger().info(
            "".join(
                [
                    "\n COMPUTED TRANSFORMATION:",
                    "\n Translation: \n",
                    str(translation),
                    "\n Rotation: \n",
                    str(rotation),
                ]
            )
        )

    def execution_round(self, poses: list[PoseStamped]) -> None:
        for pose in poses:
            self.move_to_calibration_start()
            self.move_to_pose(pose)
            self.rotate_joint(LAST_JOINT, LAST_JOINT_DEFAULT - LAST_JOINT_OFFSET)
            self.take_calibration_sample()
            self.rotate_joint(LAST_JOINT, LAST_JOINT_DEFAULT + LAST_JOINT_OFFSET)
            self.take_calibration_sample()

    def move_to_pose(self, pose: PoseStamped) -> None:
        self.add_marker.call(AddMarker.Request(marker_pose=pose))
        self.define_goal_pose.call(DefineGoalPose.Request(pose=pose))
        self.move_hand_to_pose.call(MoveHandToPose.Request())


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = Calibrate()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
