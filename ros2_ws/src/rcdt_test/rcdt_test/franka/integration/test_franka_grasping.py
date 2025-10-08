# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from _pytest.fixtures import SubRequest
from launch import LaunchDescription
from rcdt_launch.robot import Arm, Camera
from rcdt_launch.rviz import Rviz
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import call_trigger_service, wait_for_register
from rclpy.node import Node

namespace_arm = "franka"
namespace_camera = "realsense"


@launch_pytest.fixture(scope="module")
def grasping_launch(request: SubRequest) -> LaunchDescription:
    """Fixture for grasping launch file.

    Args:
        request (SubRequest): The pytest SubRequest object.

    Returns:
        LaunchDescription: The launch description for the franka core, controllers, and gripper services.
    """
    franka = Arm(
        platform="franka",
        position=[0, 0, 0],
        namespace=namespace_arm,
        gripper=True,
        moveit=True,
        graspnet=True,
    )
    Camera(
        "realsense",
        [0.05, 0, 0],
        [0, -90, 180],
        parent=franka,
        namespace=namespace_camera,
    )
    Rviz.add_markers()
    franka_launch = RegisteredLaunchDescription(
        get_file_path("rcdt_launch", ["launch"], "robots.launch.py"),
        launch_arguments={
            "rviz": "True",
            "simulation": request.config.getoption("simulation"),
            "load_gazebo_ui": "False",
            "world": "table_with_1_brick.sdf",
        },
    )
    return Register.connect_context([franka_launch])


@pytest.mark.skip(
    reason="Temporarily disabled while debugging graspnet, needs better object avoidance"
)
@pytest.mark.launch(fixture=grasping_launch)
def test_wait_for_register(timeout: int) -> None:
    """Test that the robot is registered in the system to start the tests.

    Args:
        timeout (int): The timeout in seconds before stopping the test.
    """
    wait_for_register(timeout=timeout)


@pytest.mark.skip(
    reason="Temporarily disabled while debugging graspnet, needs better object avoidance"
)
@pytest.mark.launch(fixture=grasping_launch)
def test_graspnet_request(test_node: Node, timeout: int = 100) -> None:
    """Test the grasp generation service.

    Args:
        test_node (Node): The ROS2 node to use for the service call.
        timeout (int): The timeout in seconds before stopping the test.
    """
    assert (
        call_trigger_service(
            node=test_node, service_name="/grasp/trigger", timeout=timeout
        )
        is True
    )
