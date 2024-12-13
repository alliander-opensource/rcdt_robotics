# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from PyFlow.Core import PinBase
from RCDT2.Nodes.core import Service, PyflowComputer
from geometry_msgs.msg import Transform


def get_pyflow_nodes_from_ros_messages() -> dict:
    class TranfromMessage(PyflowComputer):
        def __init__(self, name: str):
            super().__init__(name)
            self.height: PinBase = self.createInputPin("height", "FloatPin", 0)
            self.rotate_90: PinBase = self.createInputPin("rotate_90", "BoolPin", False)
            self.transform: PinBase = self.createOutputPin(
                "transfrom", "geometry_msgs/Transform", Transform()
            )

        def compute(self, *_args: any, **_kwargs: any) -> None:
            transform = Transform()
            transform.translation.z = self.height.getData()
            if self.rotate_90.getData():
                transform.rotation.w = 0.707
                transform.rotation.z = 0.707
            self.transform.setData(transform)

        @staticmethod
        def category() -> None:
            return "Messages"

    return {TranfromMessage.__name__: TranfromMessage}


def get_pyflow_nodes_from_ros_services(services: list[tuple[str, str]]) -> dict:
    return {
        service_name: create_class_from_service(service_type)
        for service_name, service_type in services
    }


def create_class_from_service(service: type) -> type:
    def init(self: object, name: str) -> None:
        """Dynamically creates an  __init__(), with correct arguments.

        A call to super() is needed, similar to:
            super(ClassTypeOfSelf, self).__init__(name)

        """
        super(type(self), self).__init__(name)

    return type(
        service.__name__, (Service,), {"__init__": init, "service_type": service}
    )
