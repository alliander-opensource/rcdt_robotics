# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa

from PyFlow.UI.Canvas.UINodeBase import UINodeBase


def createUINode(raw_instance):
    """Create a UI node from a raw instance.
    Args:
        raw_instance: The raw instance of the node to be converted into a UI node.
    Returns:
        UINodeBase: An instance of UINodeBase that represents the UI node.
    """
    return UINodeBase(raw_instance)
