# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from RCDT.Core.core import Service, Message
from RCDT.Core.definitions import ServiceDefinition


def get_pyflow_nodes_from_ros_messages(messages: list[type]) -> dict:
    return {
        message.__name__: create_class_from_message(message) for message in messages
    }


def create_class_from_message(message: type) -> type:
    return type(
        message.__name__,
        (Message,),
        {
            "__init__": lambda self, name: super(type(self), self).__init__(name),
            "message_type": message,
        },
    )


def get_pyflow_nodes_from_ros_services(
    service_definitions: list[ServiceDefinition],
) -> dict:
    return {
        service_definition.service_type.__name__: create_class_from_service(
            service_definition
        )
        for service_definition in service_definitions
    }


def create_class_from_service(service_definition: ServiceDefinition) -> type:
    return type(
        service_definition.service_type.__name__,
        (Service,),
        {
            "__init__": lambda self, name: super(type(self), self).__init__(name),
            "service_name": service_definition.service_name,
            "service_type": service_definition.service_type,
        },
    )