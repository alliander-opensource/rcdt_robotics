# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from PyFlow.Core import NodeBase, PinBase
import rclpy
from rclpy.node import Node
from threading import Thread
from time import time, sleep
from logging import getLogger
from inflection import underscore

SERVICE_AVAILABLE_TIMEOUT = 3
SERVICE_RESPONSE_TIMEOUT = 20

logger = getLogger(__name__)


class PinManager:
    def __init__(self):
        self.input_pins: dict[str, PinBase] = {}
        self.output_pins: dict[str, PinBase] = {}

    def add_input_pin(self, pin_name: str, pin: PinBase) -> None:
        self.input_pins[pin_name] = pin

    def add_output_pin(self, pin_name: str, pin: PinBase) -> None:
        self.output_pins[pin_name] = pin

    def get_data(self, pin_name: str) -> object:
        return self.input_pins[pin_name].getData()

    def set_data(self, pin_name: str, data: object) -> object:
        return self.output_pins[pin_name].setData(data)

    def get_input_pin_names(self) -> list[str]:
        return self.input_pins.keys()

    def get_output_pin_names(self) -> list[str]:
        return self.output_pins.keys()


class PyflowRosBridge:
    rclpy.init()
    node = Node("pyflow")
    Thread(target=rclpy.spin, args=[node], daemon=True).start()
    node.get_logger().info("PyFlow ROS Node started!")


class PyflowBase(NodeBase):
    def __init__(self, name: str):
        super().__init__(name)
        self.pin_manager = PinManager()


class PyflowComputer(PyflowBase):
    def compute(self, *_args: any, **_kwargs: any) -> None:
        raise NotImplementedError


class PyflowExecutor(PyflowBase):
    def execute(self, *_args: any, **_kwargs: any) -> None:
        raise NotImplementedError


class PyflowNonBlockingExecutor(PyflowExecutor):
    def __init__(self, name: str):
        super().__init__(name)
        self.non_blocking_executor_is_active = False

    def execute_parallel(self, *_args: any, **_kwargs: any) -> None:
        if self.non_blocking_executor_is_active:
            logger.error("Executor is already working. Rejected.")
            return
        thread = Thread(target=self.execution_steps)
        thread.start()

    def execution_steps(self) -> None:
        self.non_blocking_executor_is_active = True
        self.execute()
        self.non_blocking_executor_is_active = False


class Service(PyflowNonBlockingExecutor):
    def __init__(self, name: str):
        super().__init__(name)
        self.service_type: type

        self.createInputPin("In", "ExecPin", callback=self.execute_parallel)
        self.exec_out: PinBase = self.createOutputPin("Out", "ExecPin")

        for service_part in ["Request", "Response"]:
            self.create_data_pins(service_part)

        self.service_name = underscore(self.service_type.__name__)
        self.client = PyflowRosBridge.node.create_client(
            self.service_type, self.service_name
        )

    def execute(self, *_args: any, **_kwargs: any) -> None:
        request = self.create_request_from_pins()
        response = self.call_service(request)
        if not response.success:
            logger.error(
                f"Call to {self.service_name} with type {self.service_type.__name__} was unsuccessfull."
            )
            return
        logger.info("Response was succesfull.")
        self.set_response_to_pins(response)
        self.exec_out.call()

    def create_data_pins(self, service_part: str) -> None:
        msg: type = getattr(self.service_type, service_part)
        fields_and_field_types: dict[str, str] = msg.get_fields_and_field_types()
        for field, field_type in fields_and_field_types.items():
            pin_type = get_pin_type(field_type)
            if msg.__name__ == self.service_type.__name__ + "_Request":
                pin = self.createInputPin(field, pin_type)
                self.pin_manager.add_input_pin(field, pin)
            elif msg.__name__ == self.service_type.__name__ + "_Response":
                pin = self.createOutputPin(field, pin_type)
                self.pin_manager.add_output_pin(field, pin)
            else:
                logger.warning("Message is not a Request or a Response. Skip.")

    def create_request_from_pins(self) -> object:
        request = self.service_type.Request()
        for pin_name in self.pin_manager.get_input_pin_names():
            data = self.pin_manager.get_data(pin_name)
            setattr(request, pin_name, data)
        return request

    def set_response_to_pins(self, response: object) -> None:
        for pin_name in self.pin_manager.get_output_pin_names():
            data = getattr(response, pin_name)
            self.pin_manager.set_data(pin_name, data)

    def call_service(self, request: object) -> object:
        logger.info(f"Calling service {self.service_name}...")
        response = self.service_type.Response()
        response.success = False
        if not self.client.wait_for_service(SERVICE_AVAILABLE_TIMEOUT):
            logger.error("Service not available. Exit.")
            return response

        future = self.client.call_async(request)
        start_time = time()
        while time() - start_time < SERVICE_RESPONSE_TIMEOUT:
            if future.done():
                break
            sleep(1)
        else:
            future.cancel()
            logger.error("No response from service. Exit.")
            return response
        return future.result()

    @staticmethod
    def category() -> None:
        return "Services"


def get_pin_type(field_type: str) -> str:
    """
    Connect ROS built-in-types to default pins.
    https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html#field-types
    Otherwise field_type is a ROS message, of which a pin should be made automatically.
    """
    return field_type_to_pin_type.get(field_type, field_type)


field_type_to_pin_type: dict[str, str] = {
    "boolean": "BoolPin",
    "string": "StringPin",
    "wstring": "StringPin",
    "float": "FloatPin",
    "float32": "FloatPin",
    "float64": "FloatPin",
    "int": "IntPin",
    "int8": "IntPin",
    "uint8": "IntPin",
    "int16": "IntPin",
    "uint16": "IntPin",
    "int32": "IntPin",
    "uint32": "IntPin",
    "int64": "IntPin",
    "uint64": "IntPin",
}
