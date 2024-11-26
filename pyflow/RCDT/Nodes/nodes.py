from RCDT.Nodes.core import PyflowNode, RosNode

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject

from sensor_msgs.msg import Image
from rclpy import wait_for_message
from rosidl_runtime_py import convert
import json

from rcdt_utilities_msgs.action import Moveit

from rcdt_utilities_msgs.srv import AddMarker, AddObject
from rcdt_detection_msgs.srv import SegmentImage
from std_srvs.srv import Trigger


class RvizMark:
    def __init__(self, ui: RosNode):
        self.ui = ui
        ui.msg_in = PoseStamped()
        ui.client = PyflowNode.node.create_client(
            AddMarker, "/rviz_controller/add_marker"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = AddMarker.Request()
        request.marker_pose = self.ui.get_message()
        self.ui.call_service(request)


class GetImage:
    def __init__(self, ui: RosNode):
        self.ui = ui
        ui.msg_in = str()
        ui.msg_out = Image()
        ui.run_async = self.run_async

    def run_async(self) -> None:
        success, message = wait_for_message.wait_for_message(
            msg_type=Image,
            node=PyflowNode.node,
            topic=self.ui.pin_msg_in.getData(),
            time_to_wait=1,
        )
        if success:
            msg_dict = convert.message_to_ordereddict(message)
            msg_json = json.dumps(msg_dict)
            self.ui.pin_msg_out.setData(msg_json)
            self.ui.success = True


class Segment:
    def __init__(self, ui: RosNode):
        self.ui = ui
        ui.msg_in = Image()
        ui.client = PyflowNode.node.create_client(SegmentImage, "/segment_image")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = SegmentImage.Request()
        request.image = self.ui.get_message()
        request.publish = True
        self.ui.call_service(request)


class MoveitMove:
    def __init__(self, ui: RosNode):
        self.ui = ui
        ui.msg_in = PoseStamped()
        ui.client = ActionClient(PyflowNode.node, Moveit, "/moveit_controller")
        ui.run_async = self.run_async

    def run_async(self) -> None:
        goal = Moveit.Goal()
        goal.goal_pose = self.ui.get_message()
        goal.goal_pose.header.frame_id = "fr3_link0"
        self.ui.call_action(goal)


class MoveitAddObject:
    def __init__(self, ui: RosNode):
        self.ui = ui
        ui.msg_in = CollisionObject()
        ui.client = PyflowNode.node.create_client(
            AddObject, "/moveit_controller/add_object"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = AddObject.Request()
        request.object = self.ui.get_message()
        self.ui.call_service(request)


class MoveitClearObjects:
    def __init__(self, ui: RosNode):
        self.ui = ui
        ui.client = PyflowNode.node.create_client(
            Trigger, "/moveit_controller/clear_objects"
        )
        ui.run_async = self.run_async

    def run_async(self) -> None:
        request = Trigger.Request()
        self.ui.call_service(request)
