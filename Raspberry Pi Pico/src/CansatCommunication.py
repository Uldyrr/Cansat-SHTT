from machine import UART


# Values
# // Constants
RADIO_MAXBYTES = 256


class APC220:
    _serialBus: UART = None

    def __init__(self, serialBus: UART):
        self._serialBus = serialBus

    @property  # Property of the private field _serialBus with only a getter
    def SerialBus(self) -> UART:
        return self._serialBus

    def Write(self, data: str) -> int:
        return self._serialBus.write(data)

    def Read(self) -> str:
        bytesData = self._serialBus.read()
        strData = bytesData.decode("utf-8")

        return strData


class RadioCom:
    _radioComponent: APC220 = None

    def __init__(self, radioComponent: APC220):
        self._radioComponent = radioComponent

    def Send(self, data: any) -> int:
        if type(data) is not str:
            data = str(data)

        return self._radioComponent.Write(data)

    def Receive(self) -> str:
        return self._radioComponent.Read()

