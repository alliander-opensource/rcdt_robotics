# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa

from PyFlow.UI.Widgets.InputWidgets import DEFAULT_WIDGET_VARIANT


def getInputWidget(
    dataType, dataSetter, defaultValue, widgetVariant=DEFAULT_WIDGET_VARIANT, **kwds
):
    """Get the input widget for a given data type.
    Args:
        dataType: The type of the data.
        dataSetter: The setter function for the data.
        defaultValue: The default value for the input widget.
        widgetVariant: The variant of the widget (default is DEFAULT_WIDGET_VARIANT).
        **kwds: Additional keyword arguments."""
    pass
