from machine import Pin, ADC


# Constants
# // RZero constants
GASSENSOR_RZERO_CO2: float = 18.61  # NOTE: It would be best to have it calibrated in every new environment
GASSENSOR_RZERO_OZONE: float = 0.0

# // Calibration constants
class GASSENSOR_CALIBRATIONGAS:
    CO2: float = 411.9
    Oxygen: float = 0.0
    Ozone: float = 0.0

# // PPM calculation constants
GASSENSOR_PPM_A: float = 116.6020682
GASSENSOR_PPM_B: float = 2.769034857

# // Temperature and humidity model constants
GASSENSOR_COR_A: float = 0.00035
GASSENSOR_COR_B: float = 0.02718
GASSENSOR_COR_B: float = 1.39538
GASSENSOR_COR_D: float = 0.0018



class GasSensor:
    _adcPin: int = 0
    _loadResistance = 0

    def __init__(self, adcPin: int, loadResistance: float):
        self._adcPin = adcPin
        self._loadResistance = loadResistance;


    def _GetCorrectionFactor(self, airTemperature: float, airHumidity: float) -> float:


    def _GetSensorResistance(self, airTemperature: float, airHumidity: float) -> float:


    def GetResistanceZero(self, airTemperature: float, airHumidity: float) -> float:


    def GetSensorPPM(self, airTemperature: float, airHumidity: float):


