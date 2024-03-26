from CansatCore import CANSAT_ADC16BIT, Clamp, DebugLog
from machine import Pin, ADC

# Constants
# // Settings
GASSENSOR_PPMMEASUREMENTS: int = 3


# // RZero constants
class GASSENSOR_RZERO:
    """
    Contains the roughly averaged gas sensor resistance zero value for various gasses

    Note
    ----
    It would be best to have these values calibrated for every new environment
    """

    CO2: float = 112.4269
    OXYGEN: float = 0.0
    OZONE: float = 0.0


# // Calibration constants

class GASSENSOR_CALIBRATIONGAS:
    """
    Contains rough PPM values of various gasses in regular air for calibration purposes
    """

    CO2: float = 411.9
    OXYGEN: float = 0.0
    OZONE: float = 0.0


# // PPM calculation constants
GASSENSOR_PPM_A: float = 116.6020682
GASSENSOR_PPM_B: float = 2.769034857

# // Temperature and humidity model constants
GASSENSOR_COR_A: float = 0.00035
GASSENSOR_COR_B: float = 0.02718
GASSENSOR_COR_C: float = 1.39538
GASSENSOR_COR_D: float = 0.0018


def _GetCorrectionFactor(airTemperature: float, airHumidity: float) -> float:
    return GASSENSOR_COR_A * airTemperature * airTemperature - GASSENSOR_COR_B * airTemperature + GASSENSOR_COR_C - (airHumidity - 33.0) * GASSENSOR_COR_D


class GasSensor:
    _adc: ADC
    _loadResistance: float
    _zeroResistance: float
    _calibrationGas: float

    _ppmMeasurements: list = [0] * GASSENSOR_PPMMEASUREMENTS
    _ppmMeasurementsIndex: int = 0

    _calibrationR0Total: float = 0.0
    _calibrationR0Count: int = 0

    def __init__(self, adcPin: int, loadResistance: float, zeroResistance: float, calibrationGas: float):
        """
        Creates a new object of GasSensor taking in an adc pin, RL and R0.

        Parameters
        ----------
        adcPin : int
            Used by the object to read analog data from the gas sensor
        loadResistance : float
            Used by the object to calculate correct sensor resistances
        zeroResistance : float
            Used by the object to calculate PPM of a specific gas
        calibrationGas : float
            Used by the object to calculate a calibrated resistance zero
        """

        self._adc = ADC(adcPin)
        self._loadResistance = loadResistance
        self._zeroResistance = zeroResistance
        self._calibrationGas = calibrationGas

    def GetRawADC(self) -> float:
        """
        Gets the raw ADC value from the gas sensor's analog pin

        Returns
        -------
        float
            The raw ADC value as an unsigned short value (ushort)
        """

        return Clamp(self._adc.read_u16(), 1, CANSAT_ADC16BIT - 1)

    def GetSensorResistance(self, airTemperature: float, airHumidity: float) -> float:
        """
        Calculates the gas sensor's resistance based off of adc value voltage and is corrected by air temperature and humidity

        Parameters
        ----------
        airTemperature : float
            The current air temperature
        airHumidity : float
            The current relative air humidity

        Returns
        -------
        float
            The gas sensor's corrected sensor resistance in kOhm

        Note
        ----
        Both the MQ-131 and MQ-135 operate on 5V and therefore have an unsafe analog top voltage of around 4-5V!
        """

        uncorrectedSensorResistance: float = ((CANSAT_ADC16BIT / self.GetRawADC()) * 5.0 - 1.0) * self._loadResistance

        return uncorrectedSensorResistance / _GetCorrectionFactor(airTemperature, airHumidity)

    def GetResistanceZero(self, airTemperature: float, airHumidity: float) -> float:
        """
        Calculates the R0 for a specific gas

        Parameters
        ----------
        airTemperature : float
            The current air temperature
        airHumidity : float
            The current relative air humidity

        Returns
        -------
        float
            The averaged R0 value, gets more precise the longer you call this function

        Note
        ----
        Use this to calibrate a specific resistance zero ONLY in the same environment. Does not average a finite amount of resistance zero values
        """

        currentR0: float = self.GetSensorResistance(airTemperature, airHumidity) * pow((self._calibrationGas / GASSENSOR_PPM_A), (1.0 / GASSENSOR_PPM_B))

        self._calibrationR0Total += currentR0
        self._calibrationR0Count += 1

        return self._calibrationR0Total / self._calibrationR0Count

    def GetPPM(self, airTemperature: float, airHumidity: float) -> float:
        """
        Calculates the ppm for a specific gas

        Parameters
        ----------
        airTemperature : float
            The current air temperature
        airHumidity : float
            The current relative air humidity

        Returns
        -------
        float
            The ppm for a specific gas after a calibrated R0
        """
        self._ppmMeasurementsIndex = self._ppmMeasurementsIndex + 1 if self._ppmMeasurementsIndex < GASSENSOR_PPMMEASUREMENTS else 1
        self._ppmMeasurements[self._ppmMeasurementsIndex - 1] = GASSENSOR_PPM_A * pow(self.GetSensorResistance(airTemperature, airHumidity) / self._zeroResistance, -GASSENSOR_PPM_B)

        ppmMeasurementsTotal: float = 0.0

        for ppmMeasurement in self._ppmMeasurements:
            ppmMeasurementsTotal += ppmMeasurement

        return ppmMeasurementsTotal / GASSENSOR_PPMMEASUREMENTS

    def Debug(self, airTemperature: float, airHumidity: float) -> None:
        """
        Prints debug data to the console

        Parameters
        ----------
        airTemperature : float
            The current air temperature
        airHumidity : float
            The current relative air humidity
        """

        DebugLog(f"Air temperature: {airTemperature}\nAir humidity: {airHumidity}\nADC value: {self.GetRawADC()}\nSensor resistance: {self.GetSensorResistance(airTemperature, airHumidity)}\nResistance Zero: {self.GetResistanceZero(airTemperature, airHumidity)}\nPPM CO2: {self.GetPPM(airTemperature, airHumidity)}\n", "CansatGas.py")
