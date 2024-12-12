from typing import Literal
from PyFlow.Core import NodeBase, PinBase
import rclpy
from rclpy.node import Node
from threading import Thread
from logging import getLogger
from inflection import underscore

from sensor_msgs.msg import Image

logger = getLogger(__name__)


class PinManager:
    def __init__(self):
        self.input_names: list[str] = []
        self.input_pins: list[PinBase] = []

        self.output_names: list[str] = []
        self.output_pins: list[PinBase] = []

    def add_input_pin(self, name: str, pin: PinBase) -> None:
        self.input_names.append(name)
        self.input_pins.append(pin)

    def add_ouput_pin(self, name: str, pin: PinBase) -> None:
        self.output_names.append(name)
        self.output_pins.append(pin)

    def get_pin_names(self, pin_type: Literal["input", "output"]) -> list[str]:
        return getattr(self, f"{pin_type}_names")


class PyflowRosBridge:
    rclpy.init()
    node = Node("pyflow")
    Thread(target=rclpy.spin, args=[node], daemon=True).start()
    node.get_logger().info("Node started!")


class PyflowBase(NodeBase):
    def __init__(self, name: str):
        super().__init__(name)

        self.pin_manager = PinManager()

    def get_data(self, pin_name: str) -> object:
        self.pin_manager.get_pin(pin_name)


class PyflowComputer(PyflowBase):
    def __init__(self, name: str):
        super().__init__(name)

    def compute(self, *_args: any, **_kwargs: any) -> None:
        raise NotImplementedError


class PyflowExecutor(PyflowBase):
    def __init__(self, name: str):
        super().__init__(name)

    def execute(self, *_args: any, **_kwargs: any) -> None:
        raise NotImplementedError


class Service(PyflowExecutor):
    def __init__(self, name: str):
        super().__init__(name)
        self.service: type

        self.createInputPin("In", "ExecPin", callback=self.execute)
        self.createOutputPin("Out", "ExecPin")
        self.create_data_pins()

        topic_name = underscore(self.service.__name__)
        self.client = PyflowRosBridge.node.create_client(self.service, topic_name)

    def execute(self, *_args: any, **_kwargs: any) -> None:
        # Simple test on data passing:
        data: Image = self.pin_manager.input_pins[0].getData()
        logger.info(f"input = {data.header.frame_id}")
        data = Image()
        data.header.frame_id = "TEST"
        self.pin_manager.output_pins[0].setData(data)

    def create_data_pins(self) -> None:
        service_parts = ["Request", "Response"]
        for service_part in service_parts:
            msg: type = getattr(self.service, service_part)
            fields_and_field_types: dict[str, str] = msg.get_fields_and_field_types()
            for field, _field_type in fields_and_field_types.items():
                pin_type = get_pin_type(_field_type)
                if msg.__name__ == self.service.__name__ + "_Request":
                    pin = self.createInputPin(field, pin_type)
                    self.pin_manager.add_input_pin(field, pin)
                elif msg.__name__ == self.service.__name__ + "_Response":
                    pin = self.createOutputPin(field, pin_type)
                    self.pin_manager.add_ouput_pin(field, pin)

    def call_service(self, request: object) -> object | None:
        logger.info("Connecting to service...")
        if not self.client.wait_for_service(3):
            logger.error("Service not available. Exit.")
            return
        logger.info("Connected. Sending request.")
        response = self.client.call(request)
        logger.info("Finished. Response received.")
        return response

    @staticmethod
    def category() -> None:
        return "Services"


def get_pin_type(_field_type: str) -> str:
    if _field_type.startswith("sequence"):
        data_type = _field_type.removeprefix("sequence<").removesuffix(">")
        print(f"Sequence of {data_type} is not supported yet. Fallback to StringPin.")
        return "StringPin"
    match _field_type:
        case "string":
            return "StringPin"
        case "boolean":
            return "BoolPin"
        case _:
            return _field_type
