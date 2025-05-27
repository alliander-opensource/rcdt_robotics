# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from logging import getLogger
from operator import attrgetter
from threading import Thread
from time import sleep, time

import rclpy
from PyFlow.Core import NodeBase, PinBase
from rclpy.node import Node

SERVICE_AVAILABLE_TIMEOUT = 3
SERVICE_RESPONSE_TIMEOUT = 20

logger = getLogger(__name__)


class PinManager:
    """Manage PyFlow node I/O pins.

    Keeps registry of input and output `PinBase` objects for a node,
    making it easy to look up, add, and retrieve pin values.

    Attributes:
        input_pins (dict[str, PinBase]): Mapping from pin name to input pin.
        output_pins (dict[str, PinBase]): Mapping from pin name to output pin.
    """

    def __init__(self):
        """Initialize the PinManager with empty dictionaries for input and output pins."""
        self.input_pins: dict[str, PinBase] = {}
        self.output_pins: dict[str, PinBase] = {}

    def add_input_pin(self, pin_name: str, pin: PinBase) -> None:
        """Add an input pin to the manager.

        Args:
            pin_name (str): The name of the pin.
            pin (PinBase): The pin to be added.
        """
        self.input_pins[pin_name] = pin

    def add_output_pin(self, pin_name: str, pin: PinBase) -> None:
        """Add an output pin to the manager.

        Args:
            pin_name (str): The name of the pin.
            pin (PinBase): The pin to be added.
        """
        self.output_pins[pin_name] = pin

    def get_data(self, pin_name: str) -> object:
        """Get data from an input pin.

        Args:
            pin_name (str): The name of the input pin.

        Returns:
            object: The data from the input pin.
        """
        return self.input_pins[pin_name].getData()

    def set_data(self, pin_name: str, data: object) -> object:
        """Set data to an output pin.

        Args:
            pin_name (str): The name of the output pin.
            data (object): The data to be set.

        Returns:
            object: The data set to the output pin.
        """
        return self.output_pins[pin_name].setData(data)

    def get_input_pin_names(self) -> list[str]:
        """Get the names of all input pins.

        Returns:
            list[str]: A list of input pin names.
        """
        return list(self.input_pins.keys())

    def get_output_pin_names(self) -> list[str]:
        """Get the names of all output pins.

        Returns:
            list[str]: A list of output pin names.
        """
        return list(self.output_pins.keys())


class PyflowRosBridge:
    """Initialize and run a singleton ROS 2 node for PyFlow.

    Creates a global `rclpy` node named "pyflow" and spins it
    on a background thread so that all PyFlow-based nodes
    can share one ROS 2 context.

    Attributes:
        node (rclpy.node.Node): The shared ROS 2 node instance.
    """

    rclpy.init()
    node = Node("pyflow")
    Thread(target=rclpy.spin, args=[node], daemon=True).start()
    node.get_logger().info("PyFlow ROS Node started!")


class PyflowBase(NodeBase):
    """Base class for PyFlow nodes integrated with ROS 2.

    Extends `NodeBase` to include a `PinManager` for handling
    the nodes input and output pins.

    Attributes:
        pin_manager (PinManager): Manager for this nodes pins.
    """

    def __init__(self, name: str):
        """Initialize the PyFlow node with a name and a PinManager.

        Args:
            name (str): The name of the node.
        """
        super().__init__(name)
        self.pin_manager = PinManager()


class PyflowComputer(PyflowBase):
    """Abstract PyFlow node that computes data but does not execute.

    Subclasses must implement `compute()` to perform calculations
    based on input pins and write results to output pins.
    """

    def compute(self, *_args: any, **_kwargs: any) -> None:
        """Compute data based on input pins and set output pins."""
        raise NotImplementedError


class PyflowExecutor(PyflowBase):
    """Abstract PyFlow node that executes actions.

    Subclasses must implement `execute()` to perform side-effecting
    operations (e.g. calling services, publishing topics).
    """

    def execute(self, *_args: any, **_kwargs: any) -> None:
        """Execute actions based on input pins and perform side effects."""
        raise NotImplementedError


class PyflowNonBlockingExecutor(PyflowExecutor):
    """Executor that runs `execute()` in a background thread.

    Allows the node to respond to incoming signals without blocking
    the main PyFlow scheduler.

    Attributes:
        non_blocking_executor_is_active (bool): True if a background
            execution is currently in progress.
    """

    def __init__(self, name: str):
        """Initialize the non-blocking executor with a name.

        Args:
            name (str): The name of the executor.
        """
        super().__init__(name)
        self.non_blocking_executor_is_active = False

    def execute_parallel(self, *_args: any, **_kwargs: any) -> None:
        """Start the execution in a separate thread if not already active."""
        if self.non_blocking_executor_is_active:
            logger.error("Executor is already working. Rejected.")
            return
        thread = Thread(target=self.execution_steps)
        thread.start()

    def execution_steps(self) -> None:
        """Run the execution steps in a separate thread."""
        self.non_blocking_executor_is_active = True
        self.execute()
        self.non_blocking_executor_is_active = False


class Service(PyflowNonBlockingExecutor):
    """PyFlow node for making ROS 2 service calls.

    Dynamically creates input/output pins matching the
    service’s request/response message fields, then calls
    the service in a non-blocking manner.

    Attributes:
        service_type (type): The ROS 2 service type class.
        service_name (str): Name of the service to call.
        client (rclpy.client.Client): The ROS 2 service client.
        exec_out (PinBase): Execution completion pin.
    """

    def __init__(self, name: str):
        """Initialize the Service node with a name.

        Args:
            name (str): The name of the service node.
        """
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
        """Execute the service call using input pins to create the request."""
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
        """Create input/output pins for the service request or response.

        Args:
            service_part (str): Either "Request" or "Response" to specify which part of the service to create pins for.
        """
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
        """Create a service request object from input pin data.

        Returns:
            object: An instance of the service request type with data from input pins.
        """
        request = self.service_type.Request()
        for pin_name in self.pin_manager.get_input_pin_names():
            data = self.pin_manager.get_data(pin_name)
            setattr(request, pin_name, data)
        return request

    def set_response_to_pins(self, response: object) -> None:
        """Set the response data to output pins.

        Args:
            response (object): The service response object containing data to set to output pins.
        """
        for pin_name in self.pin_manager.get_output_pin_names():
            data = getattr(response, pin_name)
            self.pin_manager.set_data(pin_name, data)

    def call_service(self, request: object) -> object:
        """Call the ROS 2 service with the given request.

        Args:
            request (object): The service request object to send.

        Returns:
            object: The service response object containing the result of the service call.
        """
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
    def category() -> str:
        """Get the category of the service node.

        Returns:
            str: The category of the service node.
        """
        return "Services"


@dataclass
class MessageField:
    """Data class to represent a field in a ROS 2 message.

    Attributes:
        data (object): The data of the field, typically an instance of a ROS 2 message type.
        name (str): The name of the field.
        parents (str): The hierarchical path of parent fields, used for nested messages.
        pin_type (str): The type of pin to create for this field, based on its Python type.
    """

    data: object
    name: str = ""
    parents: str = ""
    pin_type: str = None


class Message(PyflowComputer):
    """PyFlow node for creating ROS 2 message instances.

    Dynamically creates input pins for each field in the message type,

    Atributes:
        message_type (type): The ROS 2 message type class to instantiate.
        unique_input_names (list[str]): List of unique names for input pins to avoid conflicts.
    """

    def __init__(self, name: str):
        """Initialize the Message node with a name.

        Args:
            name (str): The name of the message node.
        """
        super().__init__(name)
        self.message_type: type

        self.unique_input_names: list[str] = []

        self.create_output_pin()
        self.create_input_pins()

    def compute(self, *_args: any, **_kwargs: any) -> None:
        """Compute the message by gathering data from input pins and setting it to the output pin."""
        message = self.message_type()
        for pin_name in self.pin_manager.get_input_pin_names():
            data = self.pin_manager.get_data(pin_name)
            levels = pin_name.split("/")
            if not levels[0]:
                setattr(message, levels[-1], data)
            else:
                attribute_str = ".".join(levels[:-1])
                attribute = attrgetter(attribute_str)(message)
                setattr(attribute, levels[-1], data)
        output_pin = self.pin_manager.get_output_pin_names()[0]
        self.pin_manager.set_data(output_pin, message)

    def create_output_pin(self) -> None:
        """Create an output pin for the message type."""
        pin_name = self.message_type.__name__
        module = self.message_type.__module__
        pin_type = module.split(".")[0] + "/" + pin_name
        pin = self.createOutputPin(pin_name, pin_type)
        self.pin_manager.add_output_pin(pin_name, pin)

    def create_input_pins(self) -> None:
        """Create input pins for each field in the message type."""
        message_fields = [MessageField(self.message_type())]

        while len(message_fields) > 0:
            message_field = message_fields.pop(0)
            if hasattr(message_field.data, "get_fields_and_field_types"):
                message_fields.extend(self.get_subfields(message_field))
            else:
                self.create_pin_for_basetype(message_field)

    def create_pin_for_basetype(self, message_field: MessageField) -> None:
        """Create an input pin for a basic type field in the message.

        Args:
            message_field (MessageField): The field to create a pin for, containing data, name, parents, and pin_type.
        """
        name = message_field.name
        pin_name = self.make_unique(name)
        pin_type = message_field.pin_type
        group = message_field.parents.strip("/")

        register_name = group + "/" + name
        try:
            pin = self.createInputPin(pin_name, pin_type, group=group)
        except AttributeError:
            logger.warning(f"pin_type {pin_type} for {register_name} is unknown. Skip")
            return
        self.pin_manager.add_input_pin(register_name, pin)

    @staticmethod
    def get_subfields(message_field: MessageField) -> list[MessageField]:
        """Get subfields of a message field that are also messages.

        Args:
            message_field (MessageField): The field to extract subfields from.

        Returns:
            list[MessageField]: A list of MessageField instances representing subfields.
        """
        fields_and_field_types: dict[str, str] = (
            message_field.data.get_fields_and_field_types()
        )
        subfields = []
        for field, field_type in fields_and_field_types.items():
            python_type = str(type(getattr(message_field.data, field)))
            pin_type = get_pin_type(python_type, field_type)
            data = getattr(message_field.data, field)
            parent = message_field.parents + "/" + message_field.name
            subfields.append(MessageField(data, field, parent, pin_type))
        return subfields

    def make_unique(self, name: str) -> str:
        """Ensure the pin name is unique by appending spaces if necessary.

        Args:
            name (str): The base name for the pin.

        Returns:
            str: A unique name for the pin, ensuring no conflicts with existing names.
        """
        while name in self.unique_input_names:
            name += " "
        self.unique_input_names.append(name)
        return name

    @staticmethod
    def category() -> None:
        """Get the category of the message node.

        Returns:
            str: The category of the message node.
        """
        return "Messages"


def get_pin_type(python_type: str, field_type: str) -> str:
    """Get the appropriate PyFlow pin type based on the Python type and field type.

    Args:
        python_type (str): The string representation of the Python type.
        field_type (str): The ROS 2 field type as a string.

    Returns:
        str: The PyFlow pin type to use for this field.
    """
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
