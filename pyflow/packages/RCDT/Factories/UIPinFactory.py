# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa

from PyFlow.UI.Canvas.UIPinBase import UIPinBase


def createUIPin(owningNode, raw_instance):
    """Create a UI pin from a raw instance.
    Args:
        owningNode: The node that owns this pin.
        raw_instance: The raw instance of the pin to be converted into a UI pin.
    Returns:
        UIPinBase: An instance of UIPinBase that represents the UI pin.
    """
    return UIPinBase(owningNode, raw_instance)
