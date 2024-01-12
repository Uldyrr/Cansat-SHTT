from CansatCore import *
from CansatCommunication import RadioCom
from CansatLogger import CansatLogger
from machine import Pin, UART, I2C, ADC
from imu import MPU6050
from bmp280 import *
from dht import DHT11
from micropyGPS import MicropyGPS
import _thread
import utime


# Values
# // Sensors
class sensors:
    MPU: MPU6050 = MPU6050(I2C(1, sda=Pin(26), scl=Pin(27)))
    BMP: BMP280 = BMP280(I2C(1, sda=Pin(26), scl=Pin(27)))
    DHT: DHT11 = DHT11(Pin(9))


# // Components
class components:
    GPSSerialBus: UART = UART(1, 9600, tx=Pin(4), rx=Pin(5))
    GPS: MicropyGPS = MicropyGPS(location_formatting='dd')
    Radio: RadioCom = RadioCom(UART(0, 9600, tx=Pin(16), rx=Pin(17)))
    CansatLogger = CansatLogger()


# // Sensor data
mpuData: dict = {}


# The heart of the CanSat
def MainCycle():
    while True:
        # altitudeData = GetAltitude(sensors.BMP)
        airTemperatureData, airPressureData = GetAirTemperature(sensors.BMP), GetAirPressure(sensors.BMP)
        # accelerationData, gyroData = GetAccelerationGyro(sensors.MPU, mpuData)
        gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)
        # airHumidityData = GetAirHumidity(sensors.DHT)

        # components.Radio.Send(f"{GetBuiltInTemperature()}:{airHumidityData}\n")

        components.CansatLogger.LogData(airTemperatureData, airPressureData, gpsLatitude, gpsLongitude)

        print(gpsLatitude, gpsLongitude)

        utime.sleep(CANSAT_UPDATEHZ)


def Init():
    print("Initializing!")

    sensors.BMP.use_case(BMP280_CASE_DROP)  # Indoor use

    print("Initializing CansatCore.py!")
    InitCansatCore(sensors.BMP)

    print("Initialized!")
    print("Starting Main cycle!\n")


Init()
MainCycle()
