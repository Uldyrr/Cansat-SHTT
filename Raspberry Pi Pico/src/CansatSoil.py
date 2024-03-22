# Henrik, do your thing :D
from machine import Pin, ADC


SOIL_VOLTAGEIN = 3.3               # Volts
SOIL_REFERENCERESISTANCE = 100000  # Ohms


class SoilResistanceSensor:
    _adc: ADC = None

    def __init__(self, adcPin: int):
        self._adc = ADC(adcPin)

    def MeasureResistance(self) -> int:
        voltage_out = self._adc.read_u16()
        buffer = SOIL_VOLTAGEIN / voltage_out - 1
        measured_resistance = SOIL_REFERENCERESISTANCE * buffer

        return measured_resistance
