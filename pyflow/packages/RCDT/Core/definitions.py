# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass


@dataclass
class ServiceDefinition:
    """Definition of a service in the RCDT system.

    Attributes:
        service_name (str): The name of the service.
        service_type (type): The type of the service, typically a ROS service type.
        pyflow_name (str): The name of the PyFlow node that implements this service.
        pyflow_group (str): The group to which the PyFlow node belongs.
    """

    service_name: str
    service_type: type
    pyflow_name: str
    pyflow_group: str
