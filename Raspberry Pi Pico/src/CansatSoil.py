# Henrik, do your thing :D
from CansatCore import CANSAT_ADC16BIT
from machine import Pin, ADC


SOIL_VOLTAGEIN = 3.3               # Volts
SOIL_REFERENCERESISTANCE = 100000  # Ohms


class SoilResistanceSensor:
    _adc: ADC = None

    def __init__(self, adcPin: int):
        self._adc = ADC(adcPin)

    def MeasureResistance(self) -> int:
        raw = self._adc.read_u16()

        buffer = raw * SOIL_VOLTAGEIN
        voltage_out = buffer / CANSAT_ADC16BIT
        buffer = SOIL_VOLTAGEIN / voltage_out - 1
        measured_resistance = SOIL_REFERENCERESISTANCE * buffer

        return measured_resistance
