from machine import UART


class APC220:
    _serialBus: UART = None

    def __init__(self, serialBus: UART):
        self._serialBus = serialBus

    @property  # Property of the private field _serialBus with only a getter
    def SerialBus(self):
        return self._serialBus

    # Read methods
    def Read(self) -> str:
        bytesData = self._serialBus.read()
        strData = bytesData.decode("utf-8")

        return strData

    def ReadLine(self) -> str:
        bytesData = self._serialBus.readline()
        strData = bytesData.decode("utf-8")

        return strData

    # Write methods
    def SendDict(self, dictionary: dict) -> None:
        self._serialBus.write(str(dictionary))

    def SendString(self, string: str) -> None:
        self._serialBus.write(string)
