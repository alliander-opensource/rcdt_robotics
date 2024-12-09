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


# Nodes:
from RCDT2.Nodes.loader import get_pyflow_nodes_from_ros_services

_EXPORTERS = {}
_FOO_LIBS = {}
_NODES = get_pyflow_nodes_from_ros_services()
_PINS = {}
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
