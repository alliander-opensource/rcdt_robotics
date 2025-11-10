# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from typing import Callable, Dict

from rcdt_launch.robot import GPS, Arm, Camera, Lidar, Platform, Vehicle
from rcdt_launch.rviz import Rviz


@dataclass
class ConfigurationContext:
    """Shared parameters passed to configuration functions.

    Attributes:
        config_namespace (str): The name of the selected configuration.
        use_joystick (bool): Whether to enable joystick input.
        use_sim (bool): Whether to launch the system in simulation mode.
    """

    config_namespace: str
    use_joystick: bool
    use_sim: bool


ConfigurationFunction = Callable[[ConfigurationContext], ConfigurationContext]

PLATFORM_CONFIGS: Dict[str, ConfigurationFunction] = {}


def register_configuration(
    name: str,
) -> Callable[[ConfigurationFunction], ConfigurationFunction]:
    """Decorator to register a configuration by name.

    Args:
        name (str): The identifyer of the configuration.

    Returns:
        Callable[[ConfigurationFunction], ConfigurationFunction]:
            A decorator that registers the function under the provided name.
    """

    def wrapper(fn: ConfigurationFunction) -> ConfigurationFunction:
        PLATFORM_CONFIGS[name] = fn
        return fn

    return wrapper


@register_configuration("axis")
def config_axis(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    Platform("axis", [0, 0, 0])
    return ctx


@register_configuration("gps")
def config_gps(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    GPS("nmea", [0, 0, 0.5], ip_address="10.15.20.202")
    return ctx


@register_configuration("ouster")
def config_ouster(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    Lidar("ouster", [0, 0, 0.5])
    return ctx


@register_configuration("velodyne")
def config_velodyne(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    Lidar("velodyne", [0, 0, 0.5])
    return ctx


@register_configuration("realsense")
def config_realsense(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    Camera("realsense", [0, 0, 0.5])
    return ctx


@register_configuration("zed")
def config_zed(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    Camera("zed", [0, 0, 0.5], namespace="zed")
    return ctx


@register_configuration("franka")
def config_franka(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    if not ctx.use_sim:
        Rviz.load_motion_planning_plugin = True
    Arm("franka", [0, 0, 0], gripper=True, moveit=True, ip_address="172.16.0.2")
    return ctx


@register_configuration("franka_axis")
def config_franka_axis(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    Platform.world = "table_with_1_brick.sdf"
    Rviz.add_markers()
    Rviz.load_robot_state = True
    Rviz.load_trajectory = True
    Rviz.load_planning_scene = True
    arm = Arm("franka", [0, 0, 0], moveit=True)
    Camera("realsense", [0.05, 0, 0], [0, -90, 180], parent=arm)
    return ctx


@register_configuration("franka_double")
def config_franka_double(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    franka = Arm("franka", [0, 0, 0], [0, 0, 20])
    Platform("axis", [0, 0, 0.1], [0, 20, 0], parent=franka)
    return ctx


@register_configuration("franka_planning")
def config_franka_planning(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    ctx.use_joystick = False
    Rviz.load_motion_planning_plugin = True
    Arm("franka", [1.0, 0, 0], gripper=True, moveit=True)
    Arm("franka", [-1.0, 0, 0], gripper=True, moveit=True)
    return ctx


@register_configuration("franka_realsense")
def config_franka_realsense(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    arm = Arm("franka", [0, 0, 0], moveit=True)
    Camera("realsense", [0.05, 0, 0], [0, -90, 180], parent=arm)
    return ctx


@register_configuration("panther")
def config_panther(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    Vehicle("panther", [0, 0, 0.2], namespace="panther")
    return ctx


@register_configuration("panther_axis")
def config_panther_axis(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    vehicle = Vehicle("panther", [0, 0, 0.2], orientation=[0, 0, 90])
    Platform("axis", [0, 0, 0.2], [20, 0, 0], parent=vehicle)
    return ctx


@register_configuration("panther_gps")
def config_panther_gps(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    panther = Vehicle("panther", [0, 0, 0.2])
    Camera("realsense", [0, 0, 0.2], parent=panther)
    return ctx


@register_configuration("panther_realsense")
def config_panther_realsense(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    panther = Vehicle("panther", [0, 0, 0.2])
    GPS("nmea", [0, 0, 0.2], parent=panther)
    return ctx


@register_configuration("panther_zed")
def config_panther_zed(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    panther = Vehicle("panther", [0, 0, 0.2])
    Camera("zed", [0, 0, 0.5], parent=panther)
    return ctx


@register_configuration("panther_velodyne")
def config_panther_velodyne(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
    Lidar("velodyne", [0.13, -0.13, 0.35], parent=panther)
    return ctx


@register_configuration("panther_ouster")
def config_panther_ouster(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
    Lidar("ouster", [0.13, -0.13, 0.35], parent=panther)
    return ctx


@register_configuration("mm")
def config_mm(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    panther = Vehicle("panther", [0, 0, 0.2])
    Arm("franka", [0, 0, 0.14], gripper=True, parent=panther, moveit=True)
    return ctx


@register_configuration("mm_velodyne")
def config_mm_velodyne(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
    Arm("franka", [0, 0, 0.14], gripper=True, parent=panther, moveit=True)
    Lidar("velodyne", [0.13, -0.13, 0.35], parent=panther)
    return ctx


@register_configuration("mm_ouster")
def config_mm_ouster(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
    Arm("franka", [0, 0, 0.14], gripper=True, parent=panther, moveit=True)
    Lidar("ouster", [0.13, -0.13, 0.35], parent=panther)
    return ctx


@register_configuration("panther_and_franka")
def config_panther_and_franka(ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    Vehicle("panther", [0, -0.5, 0.2])
    Arm("franka", [0, 0.5, 0])
    return ctx


def configure_system(config_ctx: ConfigurationContext) -> ConfigurationContext:  # noqa: D103
    """Instantiates the provided platform configuration.

    Args:
        config_ctx (ConfigurationContext): Configuration context that indicates what platform
            configuration is desired.

    Returns:
        ConfigurationContext: Updated version of the configuration context.

    Raises:
        ValueError: If the provided configuration is not a valid option.
    """
    namespace = config_ctx.config_namespace
    if namespace not in PLATFORM_CONFIGS:
        raise ValueError(f"Unknown configuration: {namespace}")

    updated_ctx = PLATFORM_CONFIGS[namespace](config_ctx)
    return updated_ctx
