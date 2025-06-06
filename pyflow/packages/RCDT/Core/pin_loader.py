# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass

import distinctipy
from PyFlow.Core import PinBase
from PyFlow.Core.Common import PinOptions


@dataclass
class PinData:
    """Data class to hold information about a pin in PyFlow.

    Attributes:
        register_name (str): The name used to register the pin.
        data_type (type): The type of data the pin will handle.
        color (tuple): The color associated with the pin.
    """

    register_name: str = None
    data_type: type = None
    color: tuple = None


class MessagePin:
    """Class to hold unique message types for PyFlow pins."""

    unique_types: set[type] = set()

    def __init__(self, message_type: type):
        """Initialize a MessagePin with a specific message type."""
        MessagePin.unique_types.add(message_type)


class SequencePin:
    """Class to hold unique sequence types for PyFlow pins."""

    unique_types: set[str] = set()

    def __init__(self, sequence_type: type):
        """Initialize a SequencePin with a specific sequence type.

        Args:
            sequence_type (type): The type of the sequence, typically a list or array.
        """
        SequencePin.unique_types.add(sequence_type)


def get_pyflow_pins_from_ros_services(services: set[type]) -> dict:
    """Create PyFlow pins from ROS service types.

    Args:
        services (set[type]): A set of ROS service types.

    Returns:
        dict: A dictionary mapping pin names to PyFlow pin classes.
    """
    get_types_from_services(services)
    message_types = MessagePin.unique_types
    sequence_types = SequencePin.unique_types

    colors = get_distinct_colors(len(message_types) + len(sequence_types))

    pyflow_pins = {}
    create_pyflow_pins_from_message_types(message_types, colors, pyflow_pins)
    create_pyflow_pins_from_sequence_types(sequence_types, colors, pyflow_pins)
    return pyflow_pins


def create_pyflow_pins_from_message_types(
    message_types: set[type], colors: list[tuple], pyflow_pins: dict
) -> None:
    """Create PyFlow pins from message types.

    Args:
        message_types (set[type]): A set of unique message types.
        colors (list[tuple]): A list of colors to assign to the pins.
        pyflow_pins (dict): A dictionary to store the created PyFlow pins.
    """
    for message_type in message_types:
        pin_data = PinData()
        name = message_type.__name__
        module = message_type.__module__
        pin_data.register_name = module.split(".")[0] + "/" + name
        pin_data.data_type = message_type
        pin_data.color = colors.pop()
        pyflow_pins[pin_data.register_name] = create_pin_from_pin_data(pin_data)


def create_pyflow_pins_from_sequence_types(
    sequence_types: set[str], colors: list[tuple], pyflow_pins: dict
) -> None:
    """Create PyFlow pins from sequence types.

    Args:
        sequence_types (set[str]): A set of unique sequence types.
        colors (list[tuple]): A list of colors to assign to the pins.
        pyflow_pins (dict): A dictionary to store the created PyFlow pins.
    """
    for sequence_type in sequence_types:
        pin_data = PinData()
        pin_data.register_name = sequence_type
        pin_data.data_type = type(sequence_type, (list,), {})
        pin_data.color = colors.pop()
        pyflow_pins[pin_data.register_name] = create_pin_from_pin_data(pin_data)


def get_distinct_colors(n_colors: int) -> list[tuple[float, float, float, float]]:
    """Get a list of distinct RGBA colors.

    Args:
        n_colors (int): The number of distinct colors to generate.

    Returns:
        list[tuple[float, float, float, float]]: A list of RGBA colors, each represented as a tuple.
    """
    rgb_colors = distinctipy.get_colors(n_colors)
    rgba_colors = []
    for rgb_color in rgb_colors:
        rgba_color = []
        for element in rgb_color:
            rgba_color.append(int(element * 255))
        rgba_color.append(255)
        rgba_colors.append(tuple(rgba_color))
    return rgba_colors


def get_types_from_services(service_types: set[type]) -> None:
    """Extract message types from ROS service types.

    Args:
        service_types (set[type]): A set of ROS service types.
    """
    for service_type in service_types:
        service_parts = ["Request", "Response"]
        for service_part in service_parts:
            message: type = getattr(service_type, service_part)()
            get_types_from_message(message)


def get_types_from_message(message: type) -> None:
    """Extract message and sequence types from a ROS message.

    Args:
        message (type): A ROS message type.
    """
    exclude = ["bool", "str", "int", "float"]
    fields_and_field_types: dict[str, str] = message.get_fields_and_field_types()
    for field, _field_type in fields_and_field_types.items():
        message_type = type(getattr(message, field))
        if message_type.__name__ in exclude:
            continue
        if message_type.__name__ in {"list", "array"}:
            SequencePin(_field_type)
        else:
            MessagePin(message_type)


def create_pin_from_pin_data(pin_data: PinData) -> type:
    """Create a PyFlow pin class from PinData.

    Args:
        pin_data (PinData): The data containing information about the pin.

    Returns:
        type: A new class that inherits from `PinBase`, representing the PyFlow pin.
    """

    def init(
        self: PinBase, name: str, parent: type, direction: enumerate, **kwargs: any
    ) -> None:
        """Initialize the PyFlow pin with the given name, parent, and direction.

        Args:
            self (PinBase): The instance of the pin being initialized.
            name (str): The name of the pin.
            parent (type): The parent node type of the pin.
            direction (enumerate): The direction of the pin (input or output).
            **kwargs: Additional keyword arguments for initialization.
        """
        super(type(self), self).__init__(name, parent, direction, **kwargs)
        self.disableOptions(PinOptions.Storable)

    def supported_data_types(pin_data: PinData, *_args: any) -> tuple:
        """Return the supported data types for the pin.

        Args:
            pin_data (PinData): The data containing information about the pin.
            *_args (any): Additional arguments (not used).

        Returns:
            tuple: A tuple containing the register name of the pin.
        """
        return (pin_data.register_name,)

    def pin_data_type_hint(pin_data: PinData) -> tuple:
        """Return the data type hint for the pin.

        Args:
            pin_data (PinData): The data containing information about the pin.

        Returns:
            tuple: A tuple containing the register name and data type of the pin.
        """
        return pin_data.register_name, pin_data.data_type()

    def color(pin_data: PinData, *_args: any) -> tuple:
        """Return the color associated with the pin.

        Args:
            pin_data (PinData): The data containing information about the pin.
            *_args (any): Additional arguments (not used).

        Returns:
            tuple: A tuple representing the RGBA color of the pin.
        """
        return pin_data.color

    def internal_data_structure(pin_data: PinData) -> type:
        """Return the internal data structure type for the pin.

        Args:
            pin_data (PinData): The data containing information about the pin.

        Returns:
            type: The data type of the pin.
        """
        return pin_data.data_type

    return type(
        pin_data.register_name,
        (PinBase,),
        {
            "__init__": init,
            "IsValuePin": lambda *_args: True,
            "supportedDataTypes": lambda *args: supported_data_types(pin_data, *args),
            "pinDataTypeHint": lambda: pin_data_type_hint(pin_data),
            "color": lambda *args: color(pin_data, *args),
            "internalDataStructure": lambda: internal_data_structure(pin_data),
            "processData": lambda data: data,
        },
    )
