#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List

import rclpy
from rclpy.node import Node
from rcdt_utilities.launch_utils import spin_executor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from moveit.planning import MoveItPy, PlanningComponent, PlanningSceneMonitor
from moveit.core.robot_state import RobotState
from moveit.core.robot_model import RobotModel, JointModelGroup
from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.controller_manager import ExecutionStatus
from moveit.core.planning_scene import PlanningScene

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from rcdt_utilities_msgs.srv import ExpressPoseInOtherFrame
from rcdt_utilities_msgs.srv import AddObject, MoveHandToPose, MoveToConfiguration
from std_srvs.srv import Trigger


class MoveitControllerNode(Node):
    def __init__(self, group: str) -> None:
        name = "moveit_controller"
        super().__init__(name)

        self.group = group

        self.robot = MoveItPy(node_name="moveit_py")
        if not self.init_links(group):
            return
        self.planner: PlanningComponent = self.robot.get_planning_component(group)
        self.monitor: PlanningSceneMonitor = self.robot.get_planning_scene_monitor()

        self.create_service(
            MoveHandToPose, "~/move_hand_to_pose", self.cb_move_hand_to_pose
        )
        self.create_service(
            MoveToConfiguration,
            "~/move_to_configuration",
            self.cb_move_to_configuration,
        )
        self.create_service(MoveHandToPose, "~/pick_at_pose", self.pick_at_pose)
        self.create_service(Trigger, "~/drop", self.drop)
        self.create_service(AddObject, "~/add_object", self.add_object)
        self.create_service(Trigger, "~/clear_objects", self.clear_objects)

        cbg = MutuallyExclusiveCallbackGroup()
        self.express_pose_in_other_frame = self.create_client(
            ExpressPoseInOtherFrame, "/express_pose_in_other_frame", callback_group=cbg
        )
        self.open_gripper_client = self.create_client(
            Trigger, "/open_gripper", callback_group=cbg
        )
        self.close_gripper_client = self.create_client(
            Trigger, "/close_gripper", callback_group=cbg
        )

    def init_links(self, group: str) -> bool:
        robot_model: RobotModel = self.robot.get_robot_model()
        joint_model_group: JointModelGroup = robot_model.get_joint_model_group(group)

        end_effectors: List[JointModelGroup] = robot_model.end_effectors
        if len(end_effectors) == 0:
            self.get_logger().error("No end-effector was found. Exiting...")
            return False
        end_effector = end_effectors[0]
        if len(end_effectors) > 1:
            self.get_logger().warn(
                f"Multiple end-effector defined. First one ('{end_effector.name}') is used."
            )

        self.ee_link = end_effector.link_model_names[0]
        self.links: List[str] = joint_model_group.link_model_names

        return True

    def update_state(self) -> None:
        self.planner.set_start_state_to_current_state()
        self.state: RobotState = self.planner.get_start_state()

    def change_frame_to_world(self, pose: PoseStamped) -> PoseStamped:
        request = ExpressPoseInOtherFrame.Request()
        request.pose = pose
        request.target_frame = "world"
        response: ExpressPoseInOtherFrame.Response = (
            self.express_pose_in_other_frame.call(request)
        )
        return response.pose

    def open_gripper(self) -> None:
        request = Trigger.Request()
        response: Trigger.Response = self.open_gripper_client.call(request)
        if not response.success:
            raise Exception("Failed to open gripper.")

    def close_gripper(self) -> None:
        request = Trigger.Request()
        response: Trigger.Response = self.close_gripper_client.call(request)
        if not response.success:
            raise Exception("Failed to close gripper.")

    def move_hand_to_pose(self, pose: PoseStamped) -> bool:
        self.update_state()
        self.planner.set_goal_state(pose_stamped_msg=pose, pose_link=self.ee_link)
        success = self.plan_and_execute()
        if not success:
            raise Exception("Failed to move hand to pose.")
        return success

    def move_hand_to_configuration(self, configuration: str) -> bool:
        target_values: dict = self.planner.get_named_target_state_values(configuration)
        if not target_values:
            self.get_logger().error(f"Configuration {configuration} is unknown.")
            return False
        self.update_state()
        self.planner.set_goal_state(configuration_name=configuration)
        success = self.plan_and_execute()
        if not success:
            raise Exception("Failed to move hand to configuration.")
        return success

    def cb_move_hand_to_pose(
        self, request: MoveHandToPose.Request, response: MoveHandToPose.Response
    ) -> MoveHandToPose.Response:
        response.success = self.move_hand_to_pose(request.pose)
        return response

    def pick_at_pose(
        self, request: MoveHandToPose.Request, response: MoveHandToPose.Response
    ) -> MoveHandToPose.Response:
        pose = self.change_frame_to_world(request.pose)
        position = pose.pose.position
        response.success = True

        try:
            self.open_gripper()
            position.z += 0.15
            self.move_hand_to_pose(pose)
            position.z -= 0.07
            self.move_hand_to_pose(pose)
            self.close_gripper()
            position.z += 0.15
            self.move_hand_to_pose(pose)
            self.move_hand_to_configuration("home")
        except Exception as ex:
            self.get_logger().error(f"Failed to pick at pose: {ex}")
            return response

        response.success = True
        return response

    def drop(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            self.move_hand_to_configuration("drop")
            self.open_gripper()
            self.close_gripper()
            self.move_hand_to_configuration("home")
        except Exception as ex:
            self.get_logger().error(f"Failed to drop: {ex}")
            return response

        response.success = True
        return response

    def cb_move_to_configuration(
        self,
        request: MoveToConfiguration.Request,
        response: MoveToConfiguration.Response,
    ) -> MoveToConfiguration.Response:
        response.success = self.move_hand_to_configuration(request.configuration)
        return response

    def plan_and_execute(self) -> bool:
        plan: MotionPlanResponse = self.planner.plan()
        if not plan:
            self.get_logger().error("No motion plan was found. Aborting...")
            return False

        status: ExecutionStatus = self.robot.execute(plan.trajectory, controllers=[])
        if status.status == "SUCCEEDED":
            return True
        else:
            self.get_logger().error(f"Execution resulted in status `{status.status}`.")
            return False

    def add_object(
        self, request: AddObject.Request, response: AddObject.Response
    ) -> None:
        collision_object = request.object
        collision_object.operation = CollisionObject.ADD
        collision_object.header.frame_id = "fr3_link0"
        scene: PlanningScene
        with self.monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            current_state: RobotState = scene.current_state
        current_state.update()
        response.success = True
        return response

    def clear_objects(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> None:
        scene: PlanningScene
        with self.monitor.read_write() as scene:
            scene.remove_all_collision_objects()
            current_state: RobotState = scene.current_state
        current_state.update()
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = MoveitControllerNode("fr3_arm")
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
