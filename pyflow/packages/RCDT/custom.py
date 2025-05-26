# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from logging import getLogger

from PyFlow.Core import NodeBase, PinBase
from PyFlow.Core.Common import PinDirection, PinOptions
from PyFlow.Packages.PyFlowBase.Pins.AnyPin import AnyPin

from RCDT.Core.core import PyflowExecutor


class AnyPinCustom(AnyPin):
    """Custom AnyPin implementation for logging any data type.

    This pin can be used to log any data type without restrictions.
    It inherits from AnyPin and overrides the serialization and disconnection behavior.

    Attributes:
        name (str): The name of the pin.
        owning_node (NodeBase): The node that owns this pin.
        direction (PinDirection): The direction of the pin (input/output).
    """

    def __init__(
        self, name: str, owning_node: NodeBase, direction: PinDirection, **_kwargs: any
    ):
        """Initialize the AnyPinCustom with the given name, owning node, and direction.

        Args:
            name (str): The name of the pin.
            owning_node (NodeBase): The node that owns this pin.
            direction (PinDirection): The direction of the pin (input/output).
        """
        super().__init__(name, owning_node, direction)

    def serialize(self) -> dict:
        """Disable storable before serialization.

        Returns:
            dict: The serialized data of the pin.
        """
        self.disableOptions(PinOptions.Storable)
        return super().serialize()

    def pinDisconnected(self, _other: PinBase) -> None:  # noqa: N802
        """Reset pin_type on disconnect."""
        self.setType("AnyPinCustom")

    @staticmethod
    def internalDataStructure() -> type:  # noqa: N802
        """Return the internal data structure for this pin type.

        Returns:
            type: An empty type representing the internal data structure.
        """
        return type("AnyPinCustom", (type,), {})


class LogAnything(PyflowExecutor):
    """Node to log any data type in the RCDT system.

    This node can log any data type passed to it through the input pin.
    It inherits from PyflowExecutor and uses a custom AnyPin for input.

    Attributes:
        name (str): The name of the node.
        input_pin (PinBase): Input pin for any data type.
        exec_out (PinBase): Output pin to signal execution completion.
        logger (Logger): Logger instance for logging messages.
    """

    def __init__(self, name: str):
        """Initialize the LogAnything node with the given name.

        Args:
            name (str): The name of the node.
        """
        super().__init__(name)
        self.createInputPin("In", "ExecPin", callback=self.execute)
        self.exec_out: PinBase = self.createOutputPin("Out", "ExecPin")
        self.input_pin: PinBase = self.createInputPin("data", "AnyPinCustom")
        self.logger = getLogger("LogAnything")

    def execute(self, *_args: any, **_kwargs: any) -> None:
        """Execute the logging of data from the input pin.

        This method retrieves the data from the input pin and logs it.
        """
        data = self.input_pin.getData()
        self.logger.info(data)
        self.exec_out.call()

    @staticmethod
    def category() -> str:
        """Return the category of the node."""
        return "Custom"
