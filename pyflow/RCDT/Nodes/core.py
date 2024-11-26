from PyFlow.Core import NodeBase, PinBase
from typing import Callable
from threading import Thread
import json
import logging
from rosidl_runtime_py import set_message

import rclpy
from rclpy.node import Node, Client
from rclpy.action import ActionClient


class PyflowNode(Node):
    rclpy.init()
    node = Node("pyflow")
    Thread(target=rclpy.spin, args=[node], daemon=True).start()
    node.get_logger().info("Node started!")


class RosMessage(NodeBase):
    def __init__(self, name: str):
        super(RosMessage, self).__init__(name)
        self.compute_callback: Callable | None

    @staticmethod
    def category() -> str:
        return "Messages"

    def compute(self, *_args: any, **_kwargs: any) -> None:
        if self.compute_callback is None:
            logging.error("No compute callback defined. Exit.")
            return
        self.compute_callback()


class RosNode(NodeBase):
    def __init__(self, name: str):
        super(RosNode, self).__init__(name)
        self.msg_in: object | None
        self.msg_out: object | None

        self.client: Client | ActionClient | None
        self.run_async: Callable | None

        self.exi: PinBase = self.createInputPin("In", "ExecPin", callback=self.start)
        self.exo: PinBase = self.createOutputPin("Out", "ExecPin")

        if hasattr(self, "msg_in"):
            self.pin_msg_in: PinBase = self.createInputPin(
                type(self.msg_in).__name__, "StringPin", "{}"
            )

        if hasattr(self, "msg_out"):
            self.pin_msg_out: PinBase = self.createOutputPin(
                type(self.msg_out).__name__, "StringPin", "{}"
            )

        self.active = False
        self.finished = False
        self.success = False

    @staticmethod
    def category() -> str:
        return "Nodes"

    def fix_bytes_in_dict(self, msg_dict: dict, msg: object) -> dict:
        """
        The set_message.set_message_fields() method cannot handle bytes well.
        This method sets bytes correctly, in the first layer of the dict.
        This method needs to be expanded for handling bytes in sub-dicts of the dict.
        """
        for key, item in msg_dict.items():
            item_type = type(getattr(msg, key))
            if item_type is bytes:
                str_byte = str.encode(item)
                msg_dict[key] = str_byte
        return msg_dict

    def get_message(self) -> object:
        if not hasattr(self, "pin_msg_in"):
            logging.error("No ros_msg was set. Exit.")
            return
        msg_json = self.pin_msg_in.getData()
        msg_dict = json.loads(msg_json)
        msg_dict = self.fix_bytes_in_dict(msg_dict, self.msg_in)
        set_message.set_message_fields(self.msg_in, msg_dict)
        return self.msg_in

    def call_service(self, request: object) -> None:
        logging.info("Connecting to service...")
        if not self.client.wait_for_service(3):
            logging.error("Service not available. Exit.")
            return
        logging.info("Connected.")
        logging.info("Send request to service.")
        response = self.client.call(request)
        logging.info(f"Finished: {response}")

    def call_action(self, goal: object) -> None:
        logging.info("Connecting to action server...")
        if not self.client.wait_for_server(3):
            logging.error("Action server not available. Exit.")
            return
        logging.info("Connected.")
        logging.info("Send goal to action server.")
        result = self.client.send_goal(goal)
        logging.info(f"Finished: {result}")

    def start(self, *_args: any, **_kwargs: any) -> None:
        if self.run_async is None:
            logging.error("No callable defined to run. Exit.")
            return
        self.thread = Thread(target=self.run_async)
        self.thread.start()
        self.active = True

    def Tick(self, _delta_time: float) -> None:  # noqa: N802
        if not self.active:
            return

        self.finished = not self.thread.is_alive()
        if self.finished:
            self.active = False
            self.finished = False
            if self.success:
                self.exo.call()
                self.success = False
            return
