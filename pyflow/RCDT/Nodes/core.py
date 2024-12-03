# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from PyFlow.Core import NodeBase, PinBase
from typing import Callable, Dict, Literal
from threading import Thread
import os
import logging

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


class Base(NodeBase):
    def __init__(self, name: str, executable: bool):
        super(Base, self).__init__(name)
        self.data_folder = "/home/rcdt/rcdt_robotics/pyflow/.data/"
        if not os.path.exists(self.data_folder):
            os.makedirs(self.data_folder)

        if executable:
            self.create_execution_pins()

        self.input_dict: Dict[str, type]
        self.output_dict: Dict[str, type]
        self.input_pins = self.create_data_pins(self.input_dict, "input")
        self.output_pins = self.create_data_pins(self.output_dict, "output")

    def create_execution_pins(self) -> None:
        self.exi: PinBase = self.createInputPin("In", "ExecPin", callback=self.start)
        self.exo: PinBase = self.createOutputPin("Out", "ExecPin")

    def start(self) -> None:
        pass

    def create_data_pins(
        self, type_dict: Dict[str, type | str], pin_type: Literal["input", "output"]
    ) -> Dict[str, PinBase]:
        pin_dict = {}
        for key, value in type_dict.items():
            pin_name = key
            group_name = ""

            if value is str:
                data_type = "StringPin"
                default_value = ""
            elif value is float:
                data_type = "FloatPin"
                default_value = 0.0
            else:
                data_type = "StringPin"
                default_value = ""
                group_name = value

            if isinstance(group_name, type):
                group_name = group_name.__name__

            if pin_type == "input":
                pin = self.createInputPin(
                    pin_name, data_type, default_value, group=group_name
                )
            elif pin_type == "output":
                pin = self.createOutputPin(
                    pin_name, "StringPin", "{}", group=group_name
                )
            pin_dict[key] = pin
        return pin_dict

    def get_data(self, pin_name: str) -> object:
        if pin_name not in self.input_pins:
            return
        pin = self.input_pins[pin_name]
        if self.input_dict[pin_name] in [str, float]:
            return self.get_raw_data(pin)

        file_name = pin.getData()
        path = self.data_folder + file_name + ".pickle"
        if not os.path.exists(path):
            logging.error("Data path does not exist. Exiting.")
            return None
        with open(path, "rb") as handle:
            data = pickle.load(handle)
        return data

    def get_raw_data(self, pin: PinBase) -> str | float:
        return pin.getData()

    def set_data(self, pin_name: str, data: object) -> None:
        if pin_name not in self.output_pins:
            return
        pin = self.output_pins[pin_name]
        file_name = "".join(choice(ascii_uppercase) for i in range(12))
        path = self.data_folder + file_name + ".pickle"
        with open(path, "wb") as handle:
            pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        pin.setData(file_name)


class RosMessage(Base):
    def __init__(self, name: str):
        super(RosMessage, self).__init__(name, executable=False)
        self.compute_callback: Callable | None

    def compute(self, *_args: any, **_kwargs: any) -> None:
        if self.compute_callback is None:
            logging.error("No compute callback defined. Exit.")
            return
        self.compute_callback()

    @staticmethod
    def category() -> str:
        return "Messages"


class RosNode(Base):
    def __init__(self, name: str):
        super(RosNode, self).__init__(name, executable=True)
        self.run_async: Callable | None
        self.active = False
        self.finished = False
        self.success = False

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
        for key, value in self.request.get_fields_and_field_types().items():
            pin_type: str = value
            if "/" in pin_type:
                pin_type = pin_type.split("/")[1]
            self.input_dict[key] = pin_type

        self.response = self.service.Response()
        self.output_dict = {}
        for key, value in self.response.get_fields_and_field_types().items():
            pin_type: str = value
            if "/" in pin_type:
                pin_type = pin_type.split("/")[1]
            self.output_dict[key] = pin_type

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
