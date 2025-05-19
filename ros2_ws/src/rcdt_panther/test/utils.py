# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import time

import pytest
import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rcdt_utilities.test_utils import get_joint_position, create_ready_service_client
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped
from rcdt_messages.srv import ExpressPoseInOtherFrame


class EndToEndUtils:
    def list_controllers(
        self, node: Node, controller_manager_name: str = "/franka/controller_manager"
    ) -> list[ControllerState]:
        """Query the controller manager for all currently loaded controllers.
        Args:
            node (Node): The rclpy node used to create the service client.
            controller_manager_name (str): Name or namespace of the controller manager.
        Returns:
            List[ControllerState]: List of current controller states."""
        client = node.create_client(
            ListControllers, f"{controller_manager_name}/list_controllers"
        )
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("list_controllers service not available")

        request = ListControllers.Request()
        future: Future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        response: ListControllers.Response = future.result()
        if response is None:
            raise RuntimeError("Failed to get response from list_controllers")

        return response.controller

    def get_controller_state(
        self, controllers: list[ControllerState], controller_name: str
    ) -> str:
        """Retrieve the state of a controller by name.
        Args:
            controllers (List[ControllerState]): List of controllers.
            controller_name (str): Name of the controller to find.
        Returns:
            str: The current state of the controller.
        Raises:
            ValueError: If the controller is not found."""
        for controller in controllers:
            if controller.name == controller_name:
                return controller.state
        raise ValueError(f"Controller '{controller_name}' not found")

    def wait_until_active(
        self,
        node: Node,
        controller_name: str,
        timeout_sec: float = 90.0,
        poll_interval: float = 0.5,
        controller_manager_name: str = "/franka/controller_manager",
    ) -> bool:
        """Poll until a controller becomes 'active'.
        Args:
            node (Node): rclpy node used to call the service.
            controller_name (str): Name of the controller to wait for.
            timeout_sec (float): Timeout duration in seconds.
            poll_interval (float): Interval between polls.
            controller_manager_name (str): Controller manager service name.
        Returns:
            bool: True if the controller became active, False on timeout."""
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            controllers = self.list_controllers(node, controller_manager_name)
            try:
                state = self.get_controller_state(controllers, controller_name)
                if state == "active":
                    print("The controller is active!")
                    time.sleep(1)
                    return True
            except ValueError:
                pass

            time.sleep(poll_interval)

        return False

    def wait_until_reached_joint(
        self,
        name_space: str,
        joint: str,
        expected_value: float,
        tolerance: float = 0.025,
        timeout_sec: int = 30,
    ) -> tuple[bool, float]:
        """Wait until a joint reaches the expected value within a tolerance.
        Args:
            name_space (str): Namespace of the robot (e.g., 'franka').
            joint (str): Name of the joint to check.
            expected_value (float): Target joint value in radians.
            tolerance (float): Acceptable deviation from the expected value.
            timeout_sec (int): Timeout duration in seconds.
            poll_interval (float): Interval between joint state checks.
        Returns:
            Tuple[bool, float]: (True, joint_value) if target reached; otherwise (False"""
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            try:
                joint_value = get_joint_position(name_space=name_space, joint=joint)
                if joint_value == pytest.approx(expected_value, abs=tolerance):
                    print("The joint reached its expected value!")
                    time.sleep(0.25)
                    return (True, joint_value)
            except ValueError:
                pass

            time.sleep(0.25)
        return (False, joint_value)


def call_express_pose_in_other_frame(
    node: Node, pose: PoseStamped, target_frame: str, timeout_sec: float = 5.0
) -> ExpressPoseInOtherFrame.Response:
    """
    Calls the /express_pose_in_other_frame service.

    Args:
        node (Node): An active rclpy Node.
        pose (PoseStamped): The pose to transform.
        target_frame (str): The frame to express the pose in.
        timeout_sec (float): Timeout for waiting on service and result.

    Returns:
        ExpressPoseInOtherFrame.Response: The response containing the transformed pose.
    """

    client = create_ready_service_client(
        node, ExpressPoseInOtherFrame, "/express_pose_in_other_frame"
    )

    request = ExpressPoseInOtherFrame.Request()
    request.pose = pose
    request.target_frame = target_frame

    future: Future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    response = future.result()
    if response is None:
        raise RuntimeError("Service call failed or timed out")

    return response
