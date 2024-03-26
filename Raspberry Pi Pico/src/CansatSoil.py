from CansatCore import CANSAT_ADC16BIT, Clamp
from machine import Pin, ADC


SOIL_VOLTAGEIN: float = 3.3             # Volts
SOIL_REFERENCERESISTANCE: int = 100000  # Ohms


class SoilResistanceSensor:
    _adc: ADC = None

    def __init__(self, adcPin: int):
        self._adc = ADC(adcPin)

    def MeasureResistance(self) -> int:
        raw: int = Clamp(self._adc.read_u16(), 1, CANSAT_ADC16BIT - 1)

        voltage_out: float = SOIL_VOLTAGEIN * raw / CANSAT_ADC16BIT
        buffer: float = SOIL_VOLTAGEIN / voltage_out - 1.0
        measuredResistance: int = int(SOIL_REFERENCERESISTANCE * (1.0 / buffer))

        return measuredResistance
