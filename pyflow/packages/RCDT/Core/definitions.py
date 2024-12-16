from dataclasses import dataclass


@dataclass
class ServiceDefinition:
    service_name: str
    service_type: type
