# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa

from PyFlow.UI.UIInterfaces import IPackage

# Factories
from RCDT.Factories.UIPinFactory import createUIPin
from RCDT.Factories.PinInputWidgetFactory import getInputWidget
from RCDT.Factories.UINodeFactory import createUINode

# Nodes:
from RCDT.Nodes.loader import get_nodes

PACKAGE_NAME = "RCDT"

_EXPORTERS = {}
_FOO_LIBS = {}
_NODES = get_nodes()
_PINS = {}
_TOOLS = {}
_PREFS_WIDGETS = {}


class RCDT(IPackage):
    """RCDT pyflow package"""

    def __init__(self):
        super(RCDT, self).__init__()

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
