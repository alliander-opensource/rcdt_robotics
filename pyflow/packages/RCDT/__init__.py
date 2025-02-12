# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa

PACKAGE_NAME = "RCDT"

from PyFlow.UI.UIInterfaces import IPackage

# Factories
from RCDT.Factories.UIPinFactory import createUIPin
from RCDT.Factories.PinInputWidgetFactory import getInputWidget
from RCDT.Factories.UINodeFactory import createUINode


# Load services:
from RCDT.services import unique_service_types, service_definitions
from RCDT.messages import messages
from RCDT.custom import LogAnything, AnyPinCustom
from RCDT.change_camera import ChangeCamera
from RCDT.sequence_executor import SequenceExecutor
from RCDT.Core.node_loader import (
    get_pyflow_nodes_from_ros_services,
    get_pyflow_nodes_from_ros_messages,
)
from RCDT.Core.pin_loader import get_pyflow_pins_from_ros_services

_EXPORTERS = {}
_FOO_LIBS = {}
_NODES = {
    "LogAnything": LogAnything,
    "ChangeCamera": ChangeCamera,
    "SequenceExecutor": SequenceExecutor,
}
_NODES.update(get_pyflow_nodes_from_ros_services(service_definitions))
_NODES.update(get_pyflow_nodes_from_ros_messages(messages))
_PINS = {"AnyPinCustom": AnyPinCustom}
_PINS.update(get_pyflow_pins_from_ros_services(unique_service_types))
_TOOLS = {}
_PREFS_WIDGETS = {}


class RCDT(IPackage):
    """RCDT pyflow package"""

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
