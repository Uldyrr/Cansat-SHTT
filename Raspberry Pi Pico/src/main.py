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
    previousTick: int = utime.ticks_ms()
    currentTick: int = 0
    tickDifference: int = 0
    tickUpdateOffset: int = 0

    while True:
        utime.sleep(CANSAT_UPDATEHZ - Clamp(tickUpdateOffset * 0.001, 0, CANSAT_UPDATEHZ * 0.1))

        altitudeData = GetAltitude(sensors.BMP)
        accelerationData, gyroData = GetAccelerationGyro(sensors.MPU, mpuData)
        airTemperatureData, airPressureData = GetAirTemperature(sensors.BMP), GetAirPressure(sensors.BMP)
        gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)
        # airHumidityData = GetAirHumidity(sensors.DHT)

        components.CansatLogger.LogData(airTemperatureData, airPressureData, tickDifference / 1000, mpuData["Acceleration"],
                                        mpuData["Gyroscope"], gpsLatitude, gpsLongitude)

        # components.Radio.Send(f"{GetBuiltInTemperature()}:{airHumidityData}\n")

        print(altitudeData, gpsLatitude, gpsLongitude)

        # Evaluate tick differences
        currentTick = utime.ticks_ms()
        tickDifference = currentTick - previousTick
        previousTick = currentTick
        tickUpdateOffset = tickDifference - 1000 if tickDifference > (tickUpdateOffset + 1000) else tickUpdateOffset


def Init():
    print("Initializing!")

    sensors.BMP.use_case(BMP280_CASE_INDOOR)  # Is DROP an outdoor use case? :/

    print("Initializing CansatCore.py!")

    InitCansatCore(sensors.BMP)

    print("Initialized!")
    print("Starting MainCycle()!\n")

    MainCycle()


Init()




