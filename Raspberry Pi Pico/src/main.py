from CansatCore import *
from machine import Pin, UART, I2C, ADC
from imu import MPU6050
from bmp280 import *
from micropyGPS import MicropyGPS
from lora_e32 import LoRaE32
from lora_e32_operation_constant import ResponseStatusCode
import _thread
import utime


# Values
# // Sensors
# mpu: MPU6050 = MPU6050(I2C(1, sda=Pin(26), scl=Pin(27)))
# bmp: BMP280 = BMP280(I2C(1, sda=Pin(26), scl=Pin(27)))

# // Components
# gpsSerialBus: UART = UART(1, tx=Pin(4), rx=Pin(5))
# gps: MicropyGPS = MicropyGPS()
loRaSerialBus: UART = UART(0, tx=Pin(16), rx=Pin(17))
loRa: LoRaE32 = LoRaE32("433T30D", loRaSerialBus)
loRaAux: Pin = Pin(18, Pin.IN)

# // Sensor data
mpuData: dict = {}


# Toggles the built-in LED based off of available data
def LoRaAuxUpdate():
    while True:
        ToggleBuiltInLed(loRaAux.value())


# The heart of the CanSat
def MainCycle():
    while True:
        # accelerationData, gyroData, airTemperatureData = GetMPUAccelerationGyroTemp(mpu, bmp, mpuData)
        # airPressureData, altitudeData = GetBMPPressureAltitude(bmp, airTemperatureData)
        # gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(gps, gpsSerialBus)

        print(LoRaSendDictData(loRa, {'temp': 20, 'lufttrykk': 1000}))

        utime.sleep(CANSAT_UPDATEHZ)


def Init():
    _thread.start_new_thread(LoRaAuxUpdate, ())

    # bmp.use_case(BMP280_CASE_INDOOR)  # Innendørs


Init()
MainCycle()
