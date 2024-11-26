# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from PyFlow.UI.Canvas.UINodeBase import UINodeBase


def createUINode(raw_instance):
    return UINodeBase(raw_instance)
