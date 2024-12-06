# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from RCDT.Nodes.core import RosMessage
from geometry_msgs.msg import PoseStamped, Pose, Transform
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from vision_msgs.msg import Point2D


class Point2DMsg:
    def __init__(self, ui: RosMessage):
        self.ui = ui
        ui.compute_callback = self.compute
        ui.input_dict = {"x": float, "y": float}
        ui.output_dict = {"msg": Point2D}

    def compute(self) -> None:
        data = Point2D()
        data.x = self.ui.get_data("x")
        data.y = self.ui.get_data("y")
        self.ui.set_data("msg", data)


class TransformMsg:
    def __init__(self, ui: RosMessage):
        self.ui = ui
        ui.compute_callback = self.compute
        ui.input_dict = {
            "px": float,
            "py": float,
            "pz": float,
            "ow": float,
            "ox": float,
            "oy": float,
            "oz": float,
        }
        ui.output_dict = {"msg": Transform}

    def compute(self) -> None:
        data = Transform()
        data.translation.x = self.ui.get_data("px")
        data.translation.y = self.ui.get_data("py")
        data.translation.z = self.ui.get_data("pz")
        data.rotation.w = self.ui.get_data("ow")
        data.rotation.x = self.ui.get_data("ox")
        data.rotation.y = self.ui.get_data("oy")
        data.rotation.z = self.ui.get_data("oz")
        self.ui.set_data("msg", data)


class PoseStampedMsg:
    def __init__(self, ui: RosMessage):
        self.ui = ui
        ui.compute_callback = self.compute
        ui.input_dict = {
            "frame_id": str,
            "px": float,
            "py": float,
            "pz": float,
            "ow": float,
            "ox": float,
            "oy": float,
            "oz": float,
        }
        ui.output_dict = {"msg": PoseStamped}

    def compute(self) -> None:
        data = PoseStamped()
        data.header.frame_id = self.ui.get_data("frame_id")
        data.pose.position.x = self.ui.get_data("px")
        data.pose.position.y = self.ui.get_data("py")
        data.pose.position.z = self.ui.get_data("pz")
        data.pose.orientation.w = self.ui.get_data("ow")
        data.pose.orientation.x = self.ui.get_data("ox")
        data.pose.orientation.y = self.ui.get_data("oy")
        data.pose.orientation.z = self.ui.get_data("oz")
        self.ui.set_data("msg", data)


class CollisionObjectMsg:
    def __init__(self, ui: RosMessage):
        self.ui = ui
        ui.compute_callback = self.compute
        ui.input_dict = {
            "x": float,
            "y": float,
            "z": float,
            "d0": float,
            "d1": float,
            "d2": float,
        }
        ui.output_dict = {"msg": CollisionObject}

    def compute(self) -> None:
        data = CollisionObject()
        pose = Pose()
        pose.position.x = self.ui.get_data("x")
        pose.position.y = self.ui.get_data("y")
        pose.position.z = self.ui.get_data("z")
        data.primitive_poses.append(pose)

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        for n in range(3):
            primitive.dimensions.append(self.ui.get_data(f"d{n}"))
        data.primitives.append(primitive)
        self.ui.set_data("msg", data)
