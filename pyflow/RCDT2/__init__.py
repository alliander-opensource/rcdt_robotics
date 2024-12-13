# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa

PACKAGE_NAME = "RCDT2"

from PyFlow.UI.UIInterfaces import IPackage

# Factories
from RCDT2.Factories.UIPinFactory import createUIPin
from RCDT2.Factories.PinInputWidgetFactory import getInputWidget
from RCDT2.Factories.UINodeFactory import createUINode


# Load dynamically from services:
import inspect
from RCDT2.Nodes.node_loader import get_pyflow_nodes_from_ros_services
from RCDT2.Pins.pin_loader import get_pyflow_pins_from_ros_services
from rcdt_detection_msgs import srv as rcdt_detection_services
from rcdt_utilities_msgs import srv as rcdt_utilities_services

# Right now, the services are the services defined in the rcdt_detection_msgs package:
services: list[tuple[str, str]] = []
services.extend(inspect.getmembers(rcdt_detection_services, predicate=inspect.isclass))
services.extend(inspect.getmembers(rcdt_utilities_services, predicate=inspect.isclass))

_EXPORTERS = {}
_FOO_LIBS = {}
_NODES = get_pyflow_nodes_from_ros_services(services)
_PINS = get_pyflow_pins_from_ros_services(services)
_TOOLS = {}
_PREFS_WIDGETS = {}


class RCDT2(IPackage):
    """RCDT2 pyflow package"""

    def __init__(self):
        super().__init__()

    @staticmethod
    def GetExporters():
        return _EXPORTERS

    @staticmethod
    def GetFunctionLibraries():
        return _FOO_LIBS

    @staticmethod
    def GetNodeClasses():
        return _NODES

    @staticmethod
    def GetPinClasses():
        return _PINS

    @staticmethod
    def GetToolClasses():
        return _TOOLS

    @staticmethod
    def UIPinsFactory():
        return createUIPin

    @staticmethod
    def UINodesFactory():
        return createUINode

    @staticmethod
    def PinsInputWidgetFactory():
        return getInputWidget

    @staticmethod
    def PrefsWidgets():
        return _PREFS_WIDGETS
