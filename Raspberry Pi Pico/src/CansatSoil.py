from CansatCore import CANSAT_ADC16BIT, Clamp
from machine import Pin, ADC


SOIL_VOLTAGEIN: float = 3.3              # Volts
SOIL_REFERENCERESISTANCE: int = 100000   # Ohm
SOIL_MAXRESISTANCE: int = 10000000       # Ohm


class SoilResistanceSensor:
    _adc: ADC = None

    def __init__(self, adcPin: int):
        self._adc = ADC(adcPin)

    def MeasureResistance(self) -> float:
        raw: int = Clamp(self._adc.read_u16(), 1, CANSAT_ADC16BIT - 1)

        voltage_out: float = SOIL_VOLTAGEIN * raw / CANSAT_ADC16BIT
        buffer: float = SOIL_VOLTAGEIN / voltage_out - 1.0
        measuredResistance: float = SOIL_REFERENCERESISTANCE * (1.0 / buffer)
        measuredResistance = Clamp(measuredResistance, 1, SOIL_MAXRESISTANCE) / 1000.0  # Measure resistance in KOhm

        return measuredResistance
