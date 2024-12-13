# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from RCDT2.Nodes.core import Service


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
