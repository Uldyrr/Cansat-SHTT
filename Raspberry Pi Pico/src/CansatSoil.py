# Henrik, do your thing :D
from machine import Pin, ADC


class SoilResistanceSensor:
    def __init__(self, adcPin: int):
        self._adc = ADC(adcPin)
        self.voltage_in = 3.3  # Voltage going into sensor
        self.reference_resistance = 100000

    def MeasureResistance(self) -> int:
        voltage_out = self._adc.read_u16()
        buffer = self.voltage_in / voltage_out - 1
        measured_resistance = self.reference_resistance * buffer
        return measured_resistance
