import inspect
from RCDT2.Pins.pin_core import MessagePin

from sensor_msgs import msg as sensor_msgs
from geometry_msgs import msg as geometry_msgs

messages = []
messages.extend(inspect.getmembers(sensor_msgs, predicate=inspect.isclass))
# messages.extend(inspect.getmembers(geometry_msgs, predicate=inspect.isclass))


def get_pyflow_pins_from_ros_messages() -> dict:
    pyflow_nodes = {}
    for message in messages[6:7]:
        message_name = message[0]
        message_type = message[1]
        message_class = message_type.__module__.split(".")[0] + "/" + message_name
        pyflow_nodes[message_class] = create_class_from_message(message_type)
    return pyflow_nodes


def create_class_from_message(message: type) -> type:
    return type(message.__name__, (MessagePin,), {"__init__": init, "message": message})


def init(self: object, *args) -> None:
    super(type(self), self).__init__(*args)
    print("INIT")
    print(self.message)
