# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from RCDT.Core.core import Message, Service
from RCDT.Core.definitions import ServiceDefinition


def get_pyflow_nodes_from_ros_messages(messages: list[type]) -> dict:
    """Create PyFlow nodes from ROS message types.

    Args:
        messages (list[type]): A list of ROS message types.

    Returns:
        dict: A dictionary mapping message names to PyFlow node classes.
    """
    return {
        message.__name__: create_class_from_message(message) for message in messages
    }


def create_class_from_message(message: type) -> type:
    """Create a PyFlow node class from a ROS message type.

    Args:
        message (type): The ROS message type to create a PyFlow node for.

    Returns:
        type: A new class that inherits from `Message`, representing the PyFlow node.
    """
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
    """Create PyFlow nodes from ROS service definitions.

    Args:
        service_definitions (list[ServiceDefinition]): A list of service definitions.

    Returns:
        dict: A dictionary mapping service names to PyFlow node classes.
    """
    return {
        service_definition.pyflow_name: create_class_from_service(service_definition)
        for service_definition in service_definitions
    }


def create_class_from_service(service_definition: ServiceDefinition) -> type:
    """Create a PyFlow node class from a ROS service definition.

    Args:
        service_definition (ServiceDefinition): The service definition to create a PyFlow node for.

    Returns:
        type: A new class that inherits from `Service`, representing the PyFlow node.
    """

    def category() -> str:
        """Return the category of the service.

        Returns:
            str: The category of the service.
        """
        return service_definition.pyflow_group

    return type(
        service_definition.pyflow_name,
        (Service,),
        {
            "__init__": lambda self, name: super(type(self), self).__init__(name),
            "category": category,
            "service_name": service_definition.service_name,
            "service_type": service_definition.service_type,
        },
    )
