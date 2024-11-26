# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from PyFlow.UI.Canvas.UIPinBase import UIPinBase


def createUIPin(owningNode, raw_instance):
    return UIPinBase(owningNode, raw_instance)
