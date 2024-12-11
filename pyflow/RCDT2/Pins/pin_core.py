from PyFlow.Core import PinBase
from PyFlow.Core.Common import PinOptions


class DataWrapper:
    def __init__(self, data: object = None):
        self.data = data


class MessagePin(PinBase):
    def __init__(self, *args):
        super().__init__(*args)
        self.message: type

        self.setDefaultValue(self.message())
        self.disableOptions(PinOptions.Storable)

    def getData(self):
        data_wrapper: DataWrapper = super().getData()
        return data_wrapper.data

    @staticmethod
    def IsValuePin():
        return True

    @staticmethod
    def supportedDataTypes():
        return ("MessagePin",)

    @staticmethod
    def pinDataTypeHint():
        return "MessagePin", DataWrapper()

    @staticmethod
    def color():
        return (200, 200, 50, 255)

    @staticmethod
    def internalDataStructure():
        return DataWrapper

    @staticmethod
    def processData(data):
        return MessagePin.internalDataStructure()(data)
