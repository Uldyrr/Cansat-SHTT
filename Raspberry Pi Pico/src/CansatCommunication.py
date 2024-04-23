from machine import UART
from CansatCore import CANSAT_DATAPRECISION


class RadioCom:
    _serialBus: UART = None

    def __init__(self, serialBus: UART):
        self._serialBus = serialBus

    def Send(self, data: any) -> int:
        if type(data) is not str:
            data = str(data)

        return self._serialBus.write(data)

    def Read(self) -> str:
        bytesData: bytes = self._serialBus.read()
        strData: str = bytesData.decode("utf-8")

        return strData

    @staticmethod
    def FormatCansatData(data: any):
        """
        Formats any data that will be sent from the cansat to have a minimal yet sufficient amount of bytes
        """

        if type(data) is float:
            data = round(data, CANSAT_DATAPRECISION)

        return data

