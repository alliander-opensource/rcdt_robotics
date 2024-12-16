# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from operator import attrgetter
from PyFlow.Core import NodeBase, PinBase
import rclpy
from rclpy.node import Node
from threading import Thread
from time import time, sleep
from logging import getLogger

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
        return list(self.input_pins.keys())

    def get_output_pin_names(self) -> list[str]:
        return list(self.output_pins.keys())


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
        self.service_name: str

        self.createInputPin("In", "ExecPin", callback=self.execute_parallel)
        self.exec_out: PinBase = self.createOutputPin("Out", "ExecPin")

        for service_part in ["Request", "Response"]:
            self.create_data_pins(service_part)

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
        msg_type: type = getattr(self.service_type, service_part)
        fields_and_field_types: dict[str, str] = msg_type.get_fields_and_field_types()
        for field, field_type in fields_and_field_types.items():
            python_type = str(type(getattr(msg_type(), field)))
            pin_type = get_pin_type(python_type, field_type)
            if msg_type.__name__ == self.service_type.__name__ + "_Request":
                pin = self.createInputPin(field, pin_type)
                self.pin_manager.add_input_pin(field, pin)
            elif msg_type.__name__ == self.service_type.__name__ + "_Response":
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


@dataclass
class Submessage:
    data: object
    name: str = ""
    parents: str = ""
    pin_type: str = None


class Message(PyflowComputer):
    def __init__(self, name: str):
        super().__init__(name)
        self.message_type: type

        self.create_output_pin()
        self.create_input_pins()

    def compute(self, *_args: any, **_kwargs: any) -> None:
        message = self.message_type()
        for pin_name in self.pin_manager.get_input_pin_names():
            data = self.pin_manager.get_data(pin_name)
            levels = pin_name.split("/")
            attribute_str = ".".join(levels[:-1])
            attribute = attrgetter(attribute_str)(message)
            setattr(attribute, levels[-1], data)
        print(message)
        output_pin = self.pin_manager.get_output_pin_names()[0]
        self.pin_manager.set_data(output_pin, message)

    def create_output_pin(self) -> None:
        pin_name = self.message_type.__name__
        module = self.message_type.__module__
        pin_type = module.split(".")[0] + "/" + pin_name
        pin = self.createOutputPin(pin_name, pin_type)
        self.pin_manager.add_output_pin(pin_name, pin)

    def create_input_pins(self) -> None:
        submessages = [Submessage(self.message_type())]

        while len(submessages) > 0:
            submessage = submessages.pop(0)
            if not hasattr(submessage.data, "get_fields_and_field_types"):
                pin_name = submessage.name
                pin_type = submessage.pin_type
                group = submessage.parents.strip("/")
                pin = self.createInputPin(pin_name, pin_type, group=group)

                register_name = group + "/" + pin_name
                self.pin_manager.add_input_pin(register_name, pin)
            else:
                submessages.extend(self.get_fields_to_check(submessage))

    def get_fields_to_check(self, submessage: Submessage) -> list[Submessage]:
        fields_and_field_types: dict[str, str] = (
            submessage.data.get_fields_and_field_types()
        )
        submessages = []
        for field, field_type in fields_and_field_types.items():
            python_type = str(type(getattr(submessage.data, field)))
            pin_type = get_pin_type(python_type, field_type)
            data = getattr(submessage.data, field)
            parent = submessage.parents + "/" + submessage.name
            pin = Submessage(data, field, parent, pin_type)
            submessages.append(pin)
        return submessages

    @staticmethod
    def category() -> None:
        return "Messages"


def get_pin_type(python_type: str, field_type: str) -> str:
    match python_type:
        case "<class 'bool'>":
            return "BoolPin"
        case "<class 'str'>":
            return "StringPin"
        case "<class 'int'>":
            return "IntPin"
        case "<class 'float'>":
            return "FloatPin"
        case _:
            return field_type
