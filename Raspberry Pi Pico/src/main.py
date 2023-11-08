from CansatCore import *
from machine import Pin, UART, I2C, ADC
from imu import MPU6050
from bmp280 import *
from micropyGPS import MicropyGPS
import time

# Values
alarmBuzzerDebounce: bool = False

# // Sensors
mpu: MPU6050 = MPU6050(I2C(1, sda=Pin(26), scl=Pin(27)))
bmp: BMP280 = BMP280(I2C(1, sda=Pin(26), scl=Pin(27)))

# // Components
loRaSerialBus: UART = UART(1, 9600, tx=Pin(8), rx=Pin(9))
gpsSerialBus: UART = UART(1, 9600, tx=Pin(4), rx=Pin(5))
gps: MicropyGPS = MicropyGPS()

# // Sensor data
mpuData: dict = {}


# Init
def Init():
    bmp.use_case(BMP280_CASE_INDOOR)  # Innendørs


# The heart of the CanSat
def MainCycle():
    global alarmBuzzerDebounce

    while True:
        accelerationData, gyroData, airTemperatureData = GetMPUAccelerationGyroTemp(mpu, bmp, mpuData)
        airPressureData, altitudeData = GetBMPPressureAltitude(bmp, airTemperatureData)
        gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(gps, gpsSerialBus)

        # print(mpuData)
        print(f"Latitude: {gpsLatitude}, Longitude: {gpsLongitude}")
        # print("Air Temperature: {}C".format(airTemperatureData))
        # print("Pressure: {} Pa".format(airPressureData))
        # print("Altitude:", altitudeData)

        alarmBuzzerDebounce = not alarmBuzzerDebounce

        SetAlarmBuzzerState(alarmBuzzerDebounce)

        time.sleep(CANSAT_UPDATEHZ)


Init()
MainCycle()


