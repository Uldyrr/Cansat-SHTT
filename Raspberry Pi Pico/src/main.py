from CansatCore import *
from machine import Pin, UART, I2C, ADC
from imu import MPU6050
from bmp280 import *
from apc220 import APC220
from micropyGPS import MicropyGPS
import _thread
import utime


# Values
# // Sensors
class sensors:
    MPU: MPU6050 = MPU6050(I2C(1, sda=Pin(26), scl=Pin(27)))
    BMP: BMP280 = BMP280(I2C(1, sda=Pin(26), scl=Pin(27)))

# // Components
class components:
    GPSSerialBus: UART = UART(1, tx=Pin(4), rx=Pin(5))
    GPS: MicropyGPS = MicropyGPS()
    APC: APC220 = APC220(UART(1, tx=Pin(6), rx=Pin(7)))

# // Sensor data
mpuData: dict = {}


# The heart of the CanSat
def MainCycle():
    while True:
        accelerationData, gyroData, airTemperatureData = GetMPUAccelerationGyroTemp(sensors.MPU, sensors.BMP, mpuData)
        airPressureData, altitudeData = GetBMPPressureAltitude(sensors.BMP, airTemperatureData)
        gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)

        utime.sleep(CANSAT_UPDATEHZ)


# def Init():
    # bmp.use_case(BMP280_CASE_INDOOR)  # Indoor use


# Init()
MainCycle()
