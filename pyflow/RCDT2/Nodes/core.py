from typing import Literal
from PyFlow.Core import NodeBase, PinBase
import rclpy
from rclpy.node import Node
from threading import Thread
from logging import getLogger
from inflection import underscore

logger = getLogger(__name__)


class PinManager:
    def __init__(self):
        self.ipnut_names: list[str] = []
        self.input_pins: list[PinBase] = []

        self.output_names: list[str] = []
        self.output_pins: list[PinBase] = []

    def add_pin(
        self, pin_type: Literal["input", "output"], name: str, pin: PinBase
    ) -> None:
        setattr(self, f"{pin_type}_names", name)
        setattr(self, f"{pin_type}_pins", pin)

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
        self.create_request()

    def create_data_pins(self) -> None:
        service_parts = {"Request": "input", "Response": "output"}
        for service_part, pin_type in service_parts.items():
            msg: type = getattr(self.service, service_part)
            fields_and_field_types: dict = msg.get_fields_and_field_types()
            for field, _field_type in fields_and_field_types.items():
                if msg.__name__ == self.service.__name__ + "_Request":
                    pin = self.createInputPin(field, "StringPin", "")
                elif msg.__name__ == self.service.__name__ + "_Response":
                    pin = self.createOutputPin(field, "StringPin", "")
                self.pin_manager.add_pin(pin_type, field, pin)

    def create_request(self) -> object:
        request = self.service.Request()
        logger.info(self.pin_manager.get_pin_names("input"))

    def call_service(self, request: object) -> object | None:
        logger.info("Connecting to service...")
        if not self.client.wait_for_service(3):
            logger.error("Service not available. Exit.")
            return
        logger.info("Connected. Sending request.")
        response = self.client.call(request)
        logger.info("Finished. Response received.")
        return response
