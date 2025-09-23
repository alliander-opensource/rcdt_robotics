# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.test_utils import wait_for_register
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import CameraInfo, Image


@launch_pytest.fixture(scope="module")
def franka_and_realsense_launch(
    core_launch: RegisteredLaunchDescription,
    realsense_launch: RegisteredLaunchDescription,
) -> LaunchDescription:
    """Fixture to create launch file for the franka core, controllers, and MoveIt.

    Args:
        core_launch (RegisteredLaunchDescription): The launch description for the core.
        realsense_launch (RegisteredLaunchDescription): The launch description for realsense.

    Returns:
        LaunchDescription: The launch description for the franka core, controllers, and MoveIt.
    """
    return Register.connect_context([core_launch, realsense_launch])


@pytest.mark.launch(fixture=franka_and_realsense_launch)
def test_wait_for_register(timeout: int) -> None:
    """Test that the robot is registered in the system to start the tests.

    Args:
        timeout (int): The timeout in seconds before stopping the test.
    """
    wait_for_register(timeout=timeout)


@pytest.mark.launch(fixture=franka_and_realsense_launch)
def test_color_image_published(timeout: int) -> None:
    """Test that color images are published.

    Args:
        timeout (int): The timeout in seconds to wait for the joint states to be published.
    """
    assert_for_message(Image, "/franka/realsense/color/image_raw", timeout=timeout)


@pytest.mark.launch(fixture=franka_and_realsense_launch)
def test_color_camera_info_published(timeout: int) -> None:
    """Test that color camera info is published.

    Args:
        timeout (int): The timeout in seconds to wait for the joint states to be published.
    """
    assert_for_message(
        CameraInfo, "/franka/realsense/color/camera_info", timeout=timeout
    )


@pytest.mark.launch(fixture=franka_and_realsense_launch)
def test_depth_image_published(timeout: int) -> None:
    """Test that depth images are published.

    Args:
        timeout (int): The timeout in seconds to wait for the joint states to be published.
    """
    assert_for_message(Image, "/franka/realsense/depth/image_rect_raw", timeout=timeout)


@pytest.mark.launch(fixture=franka_and_realsense_launch)
def test_depth_camera_info_published(timeout: int) -> None:
    """Test that color camera info is published.

    Args:
        timeout (int): The timeout in seconds to wait for the joint states to be published.
    """
    assert_for_message(
        CameraInfo, "/franka/realsense/depth/camera_info", timeout=timeout
    )


@pytest.mark.launch(fixture=franka_and_realsense_launch)
def test_rgbd_published(timeout: int) -> None:
    """Test that RGBD messages are published.

    Args:
        timeout (int): The timeout in seconds to wait for the joint states to be published.
    """
    assert_for_message(RGBD, "/franka/realsense/rgbd", timeout=timeout)
