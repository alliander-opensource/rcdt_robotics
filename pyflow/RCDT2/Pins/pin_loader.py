import distinctipy
from PyFlow.Core import PinBase
from PyFlow.Core.Common import PinOptions


def get_pyflow_pins_from_ros_services(services: list[tuple[str, str]]) -> dict:
    message_types = get_unique_message_types_from_services(services)
    colors = get_distinct_colors(len(message_types))

    pyflow_pins = {}
    for message_type in message_types:
        name = message_type.__name__
        module = message_type.__module__
        pin_class = module.split(".")[0] + "/" + name
        pin_color = colors.pop()
        pyflow_pins[pin_class] = create_pin_from_message_type(message_type, pin_color)
    return pyflow_pins


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


def get_unique_message_types_from_services(
    services: list[tuple[str, str]],
) -> set[type]:
    exclude = ["bool", "str", "list"]
    message_types = set()
    for service in services:
        service_type = service[1]

        service_parts = ["Request", "Response"]
        for service_part in service_parts:
            msg: type = getattr(service_type, service_part)()
            fields_and_field_types: dict[str, str] = msg.get_fields_and_field_types()
            fields = fields_and_field_types.keys()
            for field in fields:
                message_type = type(getattr(msg, field))
                if message_type.__name__ in exclude:
                    continue
                message_types.add(message_type)
    return message_types


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
