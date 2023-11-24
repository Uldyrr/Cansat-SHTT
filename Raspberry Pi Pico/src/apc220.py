from machine import UART


class APC220():
    _serialBus: UART = None

    def __init__(self, serialBus: UART):
        self._serialBus = serialBus

    @property  # Only return the serial bus after it was provided in the constructor
    def SerialBus(self):
        return self._serialBus

    def ReadLine(self) -> str:
        return self._serialBus.readline()

    def SendDict(self, dict: dict):
        self._serialBus.write(str(dict))

    def SendString(self, string: str):
        self._serialBus.write(string)

