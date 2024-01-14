from CansatCore import *
from CansatCommunication import RadioCom
from CansatLogger import CansatLogger, LOGTYPE_MAINDATA, LOGTYPE_IMUDATA
from machine import Pin, UART, I2C, ADC
from imu import MPU6050
from bmp280 import *
from dht import DHT11
from micropyGPS import MicropyGPS
import _thread
import utime


# Values
mainDataUpdateCounter: int = 0  # Saves the total count of MainCycle updates before being reset due to us updating the main data.

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


# The heart of the CanSat
def MainCycle():
    global mainDataUpdateCounter

    while True:
        # IMU data update. Measuring and logging. (CANSAT_UPDATEHZ update time)
        accelerationData, gyroData = GetAccelerationGyro(sensors.MPU)

        components.CansatLogger.LogData(  # We save two lists instead of two dicts (saves space & we can use eval() in the visualizer).
            LOGTYPE_IMUDATA,
            [accelerationData.X, accelerationData.Y, accelerationData.Z],
            [gyroData.X, gyroData.Y, gyroData.Z]
        )

        # Main data update. Measuring, logging, and radio communicating. (1.0s update time)
        mainDataUpdateCounter += 1

        if mainDataUpdateCounter >= CANSAT_UPDATEMAINDATACOUNT:
            mainDataUpdateCounter = 0

            # altitudeData = GetAltitude(sensors.BMP)
            airTemperatureData, airPressureData = GetAirTemperature(sensors.BMP), GetAirPressure(sensors.BMP)
            gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)
            # airHumidityData = GetAirHumidity(sensors.DHT)

            components.CansatLogger.LogData(LOGTYPE_MAINDATA, airTemperatureData, airPressureData, gpsLatitude, gpsLongitude)

            # components.Radio.Send(f"{GetBuiltInTemperature()}:{airHumidityData}\n")

            print(gpsLatitude, gpsLongitude)

        utime.sleep(CANSAT_UPDATEHZ)


def Init():
    print("Initializing!")

    sensors.BMP.use_case(BMP280_CASE_DROP)  # Is DROP an outdoor use case? :/

    print("Initializing CansatCore.py!")

    InitCansatCore(sensors.BMP)

    print("Initialized!")
    print("Starting MainCycle()!\n")

    MainCycle()


Init()

