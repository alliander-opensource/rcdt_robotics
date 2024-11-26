from PyFlow.Core import PinBase
from RCDT.Nodes.core import RosMessage
from rosidl_runtime_py import convert
import json

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class PoseStampedMsg:
    def __init__(self, ui: RosMessage):
        self.msg = PoseStamped()
        ui.compute_callback = self.compute
        self.px_pin: PinBase = ui.createInputPin("px", "FloatPin")
        self.py_pin: PinBase = ui.createInputPin("py", "FloatPin")
        self.pz_pin: PinBase = ui.createInputPin("pz", "FloatPin")

        self.ow_pin: PinBase = ui.createInputPin("ow", "FloatPin")
        self.ox_pin: PinBase = ui.createInputPin("ox", "FloatPin")
        self.oy_pin: PinBase = ui.createInputPin("oy", "FloatPin")
        self.oz_pin: PinBase = ui.createInputPin("oz", "FloatPin")

        self.out: PinBase = ui.createOutputPin(
            self.msg.__class__.__name__, "StringPin", "{}"
        )

    def compute(self) -> None:
        self.msg = PoseStamped()
        self.msg.pose.position.x = self.px_pin.getData()
        self.msg.pose.position.y = self.py_pin.getData()
        self.msg.pose.position.z = self.pz_pin.getData()
        self.msg.pose.orientation.w = self.ow_pin.getData()
        self.msg.pose.orientation.x = self.ox_pin.getData()
        self.msg.pose.orientation.y = self.oy_pin.getData()
        self.msg.pose.orientation.z = self.oz_pin.getData()

        msg_dict = convert.message_to_ordereddict(self.msg)
        msg_json = json.dumps(msg_dict)
        self.out.setData(msg_json)


class CollisionObjectMsg:
    def __init__(self, ui: RosMessage):
        self.msg = CollisionObject()
        ui.compute_callback = self.compute
        self.x_pin: PinBase = ui.createInputPin("x", "FloatPin")
        self.y_pin: PinBase = ui.createInputPin("y", "FloatPin")
        self.z_pin: PinBase = ui.createInputPin("z", "FloatPin")

        for n in range(3):
            setattr(self, f"d{n}_pin", ui.createInputPin(f"d{n}", "FloatPin"))

        self.out: PinBase = ui.createOutputPin(
            self.msg.__class__.__name__, "StringPin", "{}"
        )

    def compute(self) -> None:
        self.msg = CollisionObject()
        pose = Pose()
        pose.position.x = self.x_pin.getData()
        pose.position.y = self.y_pin.getData()
        pose.position.z = self.z_pin.getData()
        self.msg.primitive_poses.append(pose)

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        for n in range(3):
            pin: PinBase = getattr(self, f"d{n}_pin")
            primitive.dimensions.append(pin.getData())
        self.msg.primitives.append(primitive)

        msg_dict = convert.message_to_ordereddict(self.msg)
        msg_json = json.dumps(msg_dict)
        self.out.setData(msg_json)
