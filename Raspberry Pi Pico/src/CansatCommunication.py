from machine import UART


class APC220:
    _serialBus: UART = None

    def __init__(self, serialBus: UART):
        self._serialBus = serialBus

    @property  # Property of the private field _serialBus with only a getter
    def SerialBus(self) -> UART:
        return self._serialBus

    def Read(self) -> str:
        bytesData = self._serialBus.read()
        strData = bytesData.decode("utf-8")

        return strData

    def Write(self, data: str) -> any:
        return self._serialBus.write(data)


class RadioCom:
    _radioComponent: APC220 = None

    def __init__(self, radioComponent: APC220):
        self._radioComponent = radioComponent


    def Send(self, data: any):
        if type(data) is not str:
            data = str(data)

        self._radioComponent.Write(data)


    def Receive(self) -> any:
        return  self._radioComponent.Read()

