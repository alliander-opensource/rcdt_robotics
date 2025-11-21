# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import itertools
from functools import partial
from typing import Callable

import launch_pytest
import numpy as np
import pytest
import rclpy
import termcolor
from _pytest.fixtures import SubRequest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from nav2_msgs.action import NavigateToPose
from rcdt_launch.platforms.lidar import Lidar
from rcdt_launch.platforms.vehicle import Vehicle
from rcdt_utilities.launch_utils import reset
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.ros_utils import get_file_path
from rcdt_utilities.test_utils import (
    assert_for_message,
    call_trigger_service,
    create_ready_action_client,
    wait_for_register,
)
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler

launch_fixtures = []


def get_tests(namespace_vehicle: str, timeout: int) -> dict:
    """Get the Nav2 tests.

    Args:
        namespace_vehicle (str): The namespace of the vehicle.
        timeout (int): The timeout in seconds.

    Returns:
        dict: A dictionary containing the tests.
    """

    def test_wait_for_register(_self: object) -> None:
        """Test that the panther core is registered in the RCDT.

        Args:
            _self (object): The test class instance.
        """
        wait_for_register(timeout=timeout)

    def test_joint_states_published(_self: object) -> None:
        """Test that the joint states are published.

        Args:
            _self (object): The test class instance.
        """
        assert_for_message(
            JointState, f"/{namespace_vehicle}/joint_states", timeout=timeout
        )

    def test_e_stop_request_reset(
        _self: object, request: SubRequest, test_node: Node
    ) -> None:
        """Test that the E-Stop request service can be called to unlock the Panther.

        Args:
            _self (object): The test class instance.
            request (SubRequest): The pytest request object, used to access command line options
            test_node (Node): The ROS 2 node to use for the test.
        """
        if request.config.getoption("simulation"):
            pytest.skip("E-Stop is not available.")  # ty: ignore[call-non-callable]
        assert (
            call_trigger_service(
                node=test_node,
                service_name=f"/{namespace_vehicle}/hardware/e_stop_reset",
                timeout=timeout,
            )
            is True
        )

    def test_nav2_goal(_self: object, test_node: Node) -> None:
        """Test that the Panther can receive and process a navigation goal.

        Args:
            _self (object): The test class instance.
            test_node (Node): The ROS 2 node to use for the test.
        """
        action_client = create_ready_action_client(
            node=test_node,
            action_type=NavigateToPose,
            action_name=f"{namespace_vehicle}/navigate_to_pose",
            timeout=timeout,
        )

        goal_msg = NavigateToPose.Goal()

        pose_stamped = PoseStamped()

        pose_stamped.header.stamp = test_node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"

        # 2) Position:
        pose_stamped.pose.position.x = 0.3
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = 0.0

        # 3) Orientation with 30 degrees (quaternion):
        yaw = 30.0 * (np.pi / 180.0)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw

        goal_msg.pose = pose_stamped

        future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(test_node, future, timeout_sec=timeout)
        goal_handle: ClientGoalHandle = future.result()

        assert goal_handle.accepted

        result_future: Future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(test_node, result_future, timeout_sec=timeout)

        result: NavigateToPose.Impl.GetResultService.Response = result_future.result()
        assert result.status == GoalStatus.STATUS_SUCCEEDED, (
            f"Navigation failed with status: {result.status}"
        )

    # Collect all test methods defined above
    tests = {
        name: obj
        for name, obj in locals().items()
        if callable(obj) and name.startswith("test_")
    }
    return tests


@launch_pytest.fixture(scope="class")
def launch_fixture_wrapper(request: SubRequest) -> Callable:
    """Wrapper fixture to select the dynamically created launch fixture.

    Args:
        request (SubRequest): The pytest request object, used to access command line options.

    Returns:
        Callable: The selected launch fixture.
    """
    return launch_fixtures.pop(0)(request)


def launch_fixture(
    request: SubRequest, platforms: tuple[str, str]
) -> LaunchDescription:
    """Fixture to launch the selected platforms.

    Args:
        request (SubRequest): The pytest request object, used to access command line options.
        platforms (tuple[str, str]): The platforms to launch.

    Returns:
        LaunchDescription: The launch description containing the platform setups.
    """
    reset()
    print(termcolor.colored(f"Start Nav2 test with {platforms}", "green"))

    vehicle_platform, lidar_platform = platforms
    vehicle = Vehicle(
        platform=vehicle_platform,
        position=[0, 0, 0.2],
        namespace=vehicle_platform,
        navigation=True,
    )
    Lidar(
        platform=lidar_platform,
        position=[0.13, -0.13, 0.35],
        parent=vehicle,
        namespace=lidar_platform,
    )
    launch = RegisteredLaunchDescription(
        get_file_path("rcdt_launch", ["launch"], "bringup.launch.py"),
        launch_arguments={
            "rviz": "False",
            "simulation": request.config.getoption("simulation"),
        },
    )
    return Register.connect_context([launch])


vehicle_list = ["panther"]
lidar_list = ["velodyne", "ouster"]

# Dynamically create test classes for each combination of platforms
for platforms in itertools.product(vehicle_list, lidar_list):
    vehicle_platform, lidar_platform = platforms

    launch_fixtures.append(partial(launch_fixture, platforms=platforms))

    test_class = type(
        f"TestNav2_{vehicle_platform}_{lidar_platform}",
        (object,),
        get_tests(vehicle_platform, timeout=150),
    )
    pytest.mark.launch(fixture=launch_fixture_wrapper)(test_class)
    globals()[test_class.__name__] = test_class
