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

# // Mission data
missionMode = MISSION_MODES.PRELAUNCH
missionPreviousAltitude = 0
missionPreviousAltitudeTrigger = 0


# The mission state status update
def MissionStateUpdate(altitudeData: float, altitudeReadSuccess: bool) -> None:
    global missionMode, missionPreviousAltitude, missionPreviousAltitudeTrigger

    if missionMode == MISSION_MODES.PRELAUNCH:
        # Check whether the cansat is above a certain valid launch altitude
        missionMode = MISSION_MODES.LAUNCH if altitudeData >= MISSION_LAUNCHALTITUDE else missionMode

    elif missionMode == MISSION_MODES.LAUNCH and altitudeData <= MISSION_LAUNCHALTITUDE:  # (I'm not sure if the cansat can land somewhere higher, hopefully not)
        # Check whether the cansat is below the launch alitude and stays under an altitude for a certain amount of time
        if abs(altitudeData - missionPreviousAltitude) > MISSION_LANDEDTHRESHOLD:
            missionPreviousAltitudeTrigger += 1
        else:
            missionPreviousAltitudeTrigger = 0
            missionPreviousAltitude = altitudeData

        missionMode = MISSION_MODES.LANDED if missionPreviousAltitudeTrigger >= MISSION_LANDEDTRIGGER else missionMode


# The heart of the Cansat
def MainCycle():
    previousTick: int = utime.ticks_ms()
    tickUpdateOffset: int = 0

    while True:
        utime.sleep(CANSAT_UPDATEHZ - Clamp(tickUpdateOffset * 0.001, 0, CANSAT_UPDATEHZ * 0.1))

        altitudeData, altitudeReadSuccess = GetAltitude(sensors.BMP)

        MissionStateUpdate(altitudeData, altitudeReadSuccess)

        # MISSION STATUS: Cansat has been launched, run all systems nominally
        if missionMode != MISSION_MODES.PRELAUNCH:
            airTemperatureData, airTemperatureReadSucces = GetAirTemperature(sensors.BMP)
            airPressureData, airPressureReadSuccess = GetAirPressure(sensors.BMP)
            accelerationData, gyroData, altitudeGyroSuccess = GetAccelerationGyro(sensors.MPU, mpuData)
            gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)
            # airHumidityData, airHumidityReadSuccess = GetAirHumidity(sensors.DHT)

            components.CansatLogger.LogData(gpsLatitude, gpsLongitude, airTemperatureData, airPressureData)

            # components.Radio.Send(f"{GetBuiltInTemperature()}:{airHumidityData}\n")

            print(gpsLatitude, gpsLongitude, airTemperatureData, altitudeData)

        if missionMode == MISSION_MODES.LANDED:
            ToggleAlarmBuzzer(True)

        # Evaluate tick differences
        currentTick = utime.ticks_ms()
        tickDifference = currentTick - previousTick
        previousTick = currentTick
        tickUpdateOffset = tickDifference - 1000 if tickDifference > (tickUpdateOffset + 1000) else tickUpdateOffset


def Init():
    print("Initializing!")

    # Standard initialization
    ToggleAlarmBuzzer(False)

    sensors.BMP.use_case(BMP280_CASE_INDOOR)  # Is DROP an outdoor use case? :/

    print("Initializing CansatCore.py!")

    # Core module initalization
    InitCansatCore(sensors.BMP)

    print("Initialized!")
    print("Starting MainCycle()!\n")

    MainCycle()


Init()

