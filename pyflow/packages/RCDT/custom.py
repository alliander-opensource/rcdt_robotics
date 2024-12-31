from PyFlow.Core import PinBase, NodeBase
from PyFlow.Core.Common import PinDirection, PinOptions
from PyFlow.Packages.PyFlowBase.Pins.AnyPin import AnyPin
from RCDT.Core.core import PyflowExecutor
from logging import getLogger


class AnyPinCustom(AnyPin):
    def __init__(
        self, name: str, owning_node: NodeBase, direction: PinDirection, **_kwargs: any
    ):
        super().__init__(name, owning_node, direction)

    def serialize(self) -> dict:
        """Disable storable before serialization."""
        self.disableOptions(PinOptions.Storable)
        return super().serialize()

    def pinDisconnected(self, _other: PinBase) -> None:  # noqa: N802
        """Reset pin_type on disconnect."""
        self.setType("AnyPinCustom")

    @staticmethod
    def internalDataStructure() -> type:  # noqa: N802
        return type("AnyPinCustom", (type,), {})


class LogAnything(PyflowExecutor):
    def __init__(self, name: str):
        super().__init__(name)
        self.createInputPin("In", "ExecPin", callback=self.execute)
        self.exec_out: PinBase = self.createOutputPin("Out", "ExecPin")
        self.input_pin: PinBase = self.createInputPin("data", "AnyPinCustom")
        self.logger = getLogger("LogAnything")

    def execute(self, *_args: any, **_kwargs: any) -> None:
        data = self.input_pin.getData()
        self.logger.info(data)
        self.exec_out.call()

    @staticmethod
    def category() -> str:
        return "Custom"
