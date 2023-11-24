from CansatCore import *
from machine import Pin, UART, I2C, ADC
from imu import MPU6050
from bmp280 import *
from micropyGPS import MicropyGPS
import _thread
import utime


# Values
# // Sensors
class Sensors():
    MPU: MPU6050 = MPU6050(I2C(1, sda=Pin(26), scl=Pin(27)))
    BMP: BMP280 = BMP280(I2C(1, sda=Pin(26), scl=Pin(27)))

# // Components
class Components():
    GPSSerialBus: UART = UART(1, tx=Pin(4), rx=Pin(5))
    GPS: MicropyGPS = MicropyGPS()

# // Sensor data
mpuData: dict = {}


# The heart of the CanSat
def MainCycle():
    while True:
        accelerationData, gyroData, airTemperatureData = GetMPUAccelerationGyroTemp(Sensors.MPU, Sensors.BMP, mpuData)
        airPressureData, altitudeData = GetBMPPressureAltitude(Sensors.BMP, airTemperatureData)
        gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(Components.GPS, Components.GPSSerialBus)

        utime.sleep(CANSAT_UPDATEHZ)


#def Init():
    # bmp.use_case(BMP280_CASE_INDOOR)  # Indoor use


#Init()
MainCycle()
