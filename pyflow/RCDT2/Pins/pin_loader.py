import distinctipy
from PyFlow.Core import PinBase
from PyFlow.Core.Common import PinOptions


def get_pyflow_pins_from_ros_services(services: list[tuple[str, str]]) -> dict:
    message_types: set[type] = set()
    sequence_types: set[str] = set()
    get_types_from_services(services, message_types, sequence_types)

    colors = get_distinct_colors(len(message_types) + len(sequence_types))

    pyflow_pins = {}
    create_pyflow_pins_from_message_types(message_types, colors, pyflow_pins)
    create_pyflow_pins_from_sequence_types(sequence_types, colors, pyflow_pins)
    return pyflow_pins


def create_pyflow_pins_from_message_types(
    message_types: set[type], colors: list[tuple], pyflow_pins: dict
) -> None:
    for message_type in message_types:
        name = message_type.__name__
        module = message_type.__module__
        pin_class = module.split(".")[0] + "/" + name
        pin_color = colors.pop()
        pyflow_pins[pin_class] = create_pin_from_message_type(message_type, pin_color)


def create_pyflow_pins_from_sequence_types(
    sequence_types: set[str], colors: list[tuple], pyflow_pins: dict
) -> None:
    for sequence_type in sequence_types:
        pin_color = colors.pop()
        print(sequence_type)
        pyflow_pins[sequence_type] = create_pin_from_sequence_type(
            sequence_type, pin_color
        )


def get_distinct_colors(n_colors: int) -> list[tuple[float, float, float, float]]:
    rgb_colors = distinctipy.get_colors(n_colors)
    rgba_colors = []
    for rgb_color in rgb_colors:
        rgba_color = []
        for element in rgb_color:
            rgba_color.append(int(element * 255))
        rgba_color.append(255)
        rgba_colors.append(tuple(rgba_color))
    return rgba_colors


def get_types_from_services(
    services: list[tuple[str, str]], message_types: set[type], sequence_types: set[str]
) -> None:
    for service in services:
        service_type = service[1]
        service_parts = ["Request", "Response"]
        for service_part in service_parts:
            message: type = getattr(service_type, service_part)()
            get_types_from_message(message, message_types, sequence_types)


def get_types_from_message(
    message: type, message_types: set[type], sequence_types: set[str]
) -> None:
    exclude = ["bool", "str"]
    fields_and_field_types: dict[str, str] = message.get_fields_and_field_types()
    for field, _field_type in fields_and_field_types.items():
        message_type = type(getattr(message, field))
        if message_type.__name__ in exclude:
            continue
        if message_type.__name__ == "list":
            sequence_types.add(_field_type)
        else:
            message_types.add(message_type)


def create_pin_from_message_type(message_type: type, pin_color: tuple) -> type:
    return type(
        message_type.__name__,
        (PinBase,),
        {
            "__init__": init,
            "IsValuePin": IsValuePin,
            "supportedDataTypes": lambda *args: supportedDataTypes(message_type, *args),
            "pinDataTypeHint": lambda: pinDataTypeHint(message_type),
            "color": lambda *args: color(pin_color, *args),
            "internalDataStructure": lambda: internalDataStructure(message_type),
            "processData": processData,
        },
    )


def create_pin_from_sequence_type(sequence_type: str, pin_color: tuple) -> type:
    return type(
        sequence_type,
        (PinBase,),
        {
            "__init__": init,
            "IsValuePin": IsValuePin,
            "supportedDataTypes": lambda *args: supportedDataTypes(list, *args),
            "pinDataTypeHint": lambda: pinDataTypeHint(list),
            "color": lambda *args: color(pin_color, *args),
            "internalDataStructure": lambda: internalDataStructure(list),
            "processData": processData,
        },
    )


# Class methods based on DemoPin:
def init(self: PinBase, name, parent, direction, **kwargs) -> None:
    super(type(self), self).__init__(name, parent, direction, **kwargs)
    self.disableOptions(PinOptions.Storable)


def IsValuePin(*args):
    return True


def supportedDataTypes(message: type, *args):
    return (message.__name__,)


def pinDataTypeHint(message: type) -> tuple:
    return message.__name__, message()


def color(pin_color: tuple, *args):
    return pin_color


def internalDataStructure(message: type) -> type:
    return message


def processData(data):
    return data
