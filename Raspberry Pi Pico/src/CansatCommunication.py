from CansatLogger import CansatLogger
from machine import UART


_cansatLogger: CansatLogger = CansatLogger()


class RadioCom:
    _serialBus: UART = None

    def __init__(self, serialBus: UART):
        self._serialBus = serialBus

    def Send(self, data: any) -> int:
        if type(data) is not str:
            data = str(data)

        _cansatLogger.LogData(data)

        return self._serialBus.write(data)

    def Read(self) -> str:
        bytesData = self._serialBus.read()
        strData = bytesData.decode("utf-8")

        return strData


