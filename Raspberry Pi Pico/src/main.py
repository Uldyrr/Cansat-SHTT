from CansatCore import *
from machine import Pin, UART, I2C, ADC
from imu import MPU6050
from bmp280 import *
from micropyGPS import MicropyGPS
from lora_e32 import LoRaE32
import time

# Values
alarmBuzzerDebounce: bool = False

# // Sensors
#mpu: MPU6050 = MPU6050(I2C(1, sda=Pin(26), scl=Pin(27)))
#bmp: BMP280 = BMP280(I2C(1, sda=Pin(26), scl=Pin(27)))

# // Components
#gpsSerialBus: UART = UART(1, tx=Pin(4), rx=Pin(5))
#gps: MicropyGPS = MicropyGPS()
loRaSerialBus: UART = UART(0, tx=Pin(8), rx=Pin(9))
loRa: LoRaE32 = LoRaE32("433T30D", loRaSerialBus)

# // Sensor data
mpuData: dict = {}


# Init
def Init():
    bmp.use_case(BMP280_CASE_INDOOR)  # Innendørs


# The heart of the CanSat
def MainCycle():
    global alarmBuzzerDebounce

    while True:
        #accelerationData, gyroData, airTemperatureData = GetMPUAccelerationGyroTemp(mpu, bmp, mpuData)
        #airPressureData, altitudeData = GetBMPPressureAltitude(bmp, airTemperatureData)
        #gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(gps, gpsSerialBus)

        # print(mpuData)
        # print(f"Latitude: {gpsLatitude}, Longitude: {gpsLongitude}")
        # print("Air Temperature: {}C".format(airTemperatureData))
        # print("Pressure: {} Pa".format(airPressureData))
        # print("Altitude:", altitudeData)

        alarmBuzzerDebounce = not alarmBuzzerDebounce

        SetAlarmBuzzerState(alarmBuzzerDebounce)

        time.sleep(CANSAT_UPDATEHZ)


Init()
MainCycle()
