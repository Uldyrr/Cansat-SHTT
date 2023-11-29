from machine import UART


class APC220:
    _serialBus: UART = None

    def __init__(self, serialBus: UART):
        self._serialBus = serialBus

    @property  # Property of the private field _serialBus with only a getter
    def SerialBus(self):
        return self._serialBus

    def Read(self) -> str:
        bytesData = self._serialBus.read()
        strData = "" if bytesData is None else bytesData.decode("utf-8")

        return strData

    def ReadLine(self) -> str:
        bytesData = self._serialBus.readline()
        strData = "" if bytesData is None else bytesData.decode("utf-8")

        return strData

    def SendDict(self, dictionary: dict):
        self._serialBus.write(str(dictionary))

    def SendString(self, string: str):
        self._serialBus.write(string)
