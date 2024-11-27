# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from PyFlow.Core import NodeBase, PinBase
from typing import Callable, Dict
from threading import Thread
import json
import logging
from rosidl_runtime_py import set_message

import pickle
from random import choice
from string import ascii_uppercase

import rclpy
from rclpy.node import Node, Client


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
        self.run_async: Callable | None
        self.exi: PinBase = self.createInputPin("In", "ExecPin", callback=self.start)
        self.exo: PinBase = self.createOutputPin("Out", "ExecPin")
        self.data_folder = "/home/rcdt/rcdt_robotics/pyflow/.data/"

        self.input_dict: Dict[str, type | str]
        self.input_objects: Dict[str, object]
        self.output_dict: Dict[str, type | str]
        self.ouput_objects: Dict[str, object]

        self.input_pins: Dict[str, PinBase] = {}
        self.output_pins: Dict[str, PinBase] = {}

        self.create_pins()

        self.active = False
        self.finished = False
        self.success = False

    def create_pins(self) -> None:
        for key, value in self.input_dict.items():
            pin_name = value
            if isinstance(pin_name, type):
                pin_name = self.type_to_str(value)
            pin = self.createInputPin(pin_name, "StringPin", "{}", group=key)
            self.input_pins[key] = pin

        for key, value in self.output_dict.items():
            pin_name = value
            if isinstance(pin_name, type):
                pin_name = self.type_to_str(value)
            pin = self.createOutputPin(pin_name, "StringPin", "{}", group=key)
            self.output_pins[key] = pin

    def type_to_str(self, t: type) -> str:
        module_name = t.__module__.split(".")[0]
        name = t.__name__
        return f"{module_name}/{name}"

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

    def get_string(self, pin_name: str) -> str:
        pin = self.input_pins[pin_name]
        return pin.getData()

    def get_message(self, pin_name: str) -> object:
        pin = self.input_pins[pin_name]
        msg = self.input_objects[pin_name]
        msg_json = pin.getData()
        msg_dict = json.loads(msg_json)
        msg_dict = self.fix_bytes_in_dict(msg_dict, msg)
        set_message.set_message_fields(msg, msg_dict)
        return msg

    def get_data(self, pin_name: str) -> object:
        if pin_name not in self.input_pins:
            return
        pin = self.input_pins[pin_name]
        file_name = pin.getData()
        path = self.data_folder + file_name + ".pickle"
        with open(path, "rb") as handle:
            data = pickle.load(handle)
        return data

    def set_data(self, pin_name: str, data: object) -> None:
        if pin_name not in self.output_pins:
            return
        pin = self.output_pins[pin_name]
        file_name = "".join(choice(ascii_uppercase) for i in range(12))
        path = self.data_folder + file_name + ".pickle"
        with open(path, "wb") as handle:
            pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        pin.setData(file_name)

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

    @staticmethod
    def category() -> str:
        return "Nodes"


class RosService(RosNode):
    def __init__(self, name: str):
        self.service: object | None
        self.client: Client | None
        self.set_dicts_from_service()
        super(RosService, self).__init__(name)

    def set_dicts_from_service(self) -> None:
        self.request = self.service.Request()
        self.input_dict = {}
        self.input_objects = {}
        for key, value in self.request.get_fields_and_field_types().items():
            self.input_dict[key] = value
            self.input_objects[key] = getattr(self.request, key)

        self.response = self.service.Response()
        self.output_dict = {}
        self.output_objects = {}
        for key, value in self.response.get_fields_and_field_types().items():
            self.output_dict[key] = value
            self.output_objects[key] = getattr(self.response, key)

    def call_service(self, request: object) -> object | None:
        logging.info("Connecting to service...")
        if not self.client.wait_for_service(3):
            logging.error("Service not available. Exit.")
            return
        logging.info("Connected.")
        logging.info("Send request to service.")
        response = self.client.call(request)
        logging.info("Finished.")
        return response
