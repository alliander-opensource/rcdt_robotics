from RCDT2.Nodes.core import Service


def get_pyflow_nodes_from_ros_services(services: list[tuple[str, str]]) -> dict:
    pyflow_nodes = {}
    for service in services:
        service_name = service[0]
        service_type = service[1]
        pyflow_nodes[service_name] = create_class_from_service(service_type)
    return pyflow_nodes


def create_class_from_service(service: type) -> type:
    return type(service.__name__, (Service,), {"__init__": init, "service": service})


def init(self: object, name: str) -> None:
    super(type(self), self).__init__(name)
