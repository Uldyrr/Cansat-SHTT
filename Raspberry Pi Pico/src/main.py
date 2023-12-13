from CansatCore import *
from CansatCommunication import APC220, RadioCom
from CansatLogger import CansatLogger
from machine import Pin, UART, I2C, ADC
from imu import MPU6050
from bmp280 import *
from micropyGPS import MicropyGPS
import _thread
import utime


# Values
# // Sensors
#class sensors:
    # MPU: MPU6050 = MPU6050(I2C(1, sda=Pin(26), scl=Pin(27)))
    # BMP: BMP280 = BMP280(I2C(1, sda=Pin(26), scl=Pin(27)))

# // Components
class components:
    GPSSerialBus: UART = UART(1, tx=Pin(4), rx=Pin(5))
    APC: APC220 = APC220(UART(0, 9600, tx=Pin(16), rx=Pin(17)))

    GPS: MicropyGPS = MicropyGPS()
    Radio: RadioCom = RadioCom(APC)
    CansatLogger: CansatLogger = CansatLogger()


# // Sensor data
mpuData: dict = {}


# The heart of the CanSat
def MainCycle():
    while True:
        #accelerationData, gyroData, airTemperatureData = GetMPUAccelerationGyroTemp(sensors.MPU, sensors.BMP, mpuData)
        #airPressureData, altitudeData = GetBMPPressureAltitude(sensors.BMP, airTemperatureData)
        #gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)

        components.Radio.Send("Hello")

        utime.sleep(CANSAT_UPDATEHZ)


def Init():
    print("Starting!\n")

    # bmp.use_case(BMP280_CASE_INDOOR)  # Indoor use


Init()
MainCycle()

