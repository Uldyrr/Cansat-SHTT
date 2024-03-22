# Henrik, do your thing :D
from CansatCore import CANSAT_ADC16BIT, Clamp
from machine import Pin, ADC


SOIL_VOLTAGEIN = 3.3               # Volts
SOIL_REFERENCERESISTANCE = 100000  # Ohms


class SoilResistanceSensor:
    _adc: ADC = None

    def __init__(self, adcPin: int):
        self._adc = ADC(adcPin)

    def MeasureResistance(self) -> int:
        raw = Clamp(self._adc.read_u16(), 1, CANSAT_ADC16BIT)

        voltage_out: float = SOIL_VOLTAGEIN * raw / CANSAT_ADC16BIT
        buffer: float = SOIL_VOLTAGEIN / voltage_out - 1
        measured_resistance: int = int(SOIL_REFERENCERESISTANCE * buffer)

        return measured_resistance
