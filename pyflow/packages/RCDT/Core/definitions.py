# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass


@dataclass
class ServiceDefinition:
    service_name: str
    service_type: type
