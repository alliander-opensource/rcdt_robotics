from PyFlow.Core import PinBase
from PyFlow.Core.Common import PinOptions


class DataWrapper:
    def __init__(self, data: object = None):
        self.data = data


class DemoPin(PinBase):
    """doc string for DemoPin"""

    def __init__(self, name, parent, direction, **kwargs):
        super(DemoPin, self).__init__(name, parent, direction, **kwargs)
        self.setDefaultValue(DataWrapper())
        self.disableOptions(PinOptions.Storable)

    def getData(self):
        data_wrapper: DataWrapper = super().getData()
        return data_wrapper.data

    @staticmethod
    def IsValuePin():
        return True

    @staticmethod
    def supportedDataTypes():
        return ("DemoPin",)

    @staticmethod
    def pinDataTypeHint():
        return "DemoPin", DataWrapper()

    @staticmethod
    def color():
        return (200, 200, 50, 255)

    @staticmethod
    def internalDataStructure():
        return DataWrapper

    @staticmethod
    def processData(data):
        return DemoPin.internalDataStructure()(data)
