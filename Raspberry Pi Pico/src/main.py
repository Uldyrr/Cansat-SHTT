from CansatCore import *
from CansatCommunication import RadioCom
from CansatLogger import CansatLogger
from CansatGas import GasSensor, GASSENSOR_CALIBRATIONGAS, GASSENSOR_RZERO
from CansatSoil import SoilResistanceSensor
from machine import Pin, ADC, UART, I2C
from imu import MPU6050
from bmp280 import *
from dht import DHT11
from micropyGPS import MicropyGPS
from Servo import Servo
import utime


# Values
# // Sensors
class sensors:
    MPU: MPU6050 = MPU6050(I2C(1, sda=Pin(18), scl=Pin(19)))
    BMP: BMP280 = BMP280(I2C(1, sda=Pin(18), scl=Pin(19)))
    DHT: DHT11 = DHT11(Pin(9))
    MQ135: GasSensor = GasSensor(27, 1.0, GASSENSOR_RZERO.OXYGEN, GASSENSOR_CALIBRATIONGAS.OXYGEN)
    MQ131: GasSensor = GasSensor(26, 1.0, GASSENSOR_RZERO.OZONE, GASSENSOR_CALIBRATIONGAS.OZONE)
    SoilResistance: SoilResistanceSensor = SoilResistanceSensor(28)


# // Components
class components:
    SoilResistanceServo: Servo = Servo(8)
    GPSSerialBus: UART = UART(1, 9600, tx=Pin(4), rx=Pin(5))
    GPS: MicropyGPS = MicropyGPS(location_formatting='dd')
    Radio: RadioCom = RadioCom(UART(0, 9600, tx=Pin(16), rx=Pin(17)))
    CansatLogger = CansatLogger()


# // Mission data
missionMode: int = MISSION_MODES.PRELAUNCH
missionAltitudeFailed: bool = False
missionAltitudePrevious: float = 0.0
missionAltitudeMax: float = 0.0
missionLandedTrigger: int = 0


# The mission state status update
def MissionStateUpdate() -> None:
    global missionMode, missionAltitudeFailed, missionAltitudePrevious, missionAltitudeMax, missionLandedTrigger

    altitudeData, altitudeReadSuccess = GetAltitude(sensors.BMP)

    # Stop any mission mode updates and inform the ground station of altitude read failure
    if not altitudeReadSuccess:
        missionAltitudeFailed = True
        missionMode = MISSION_MODES.LANDED
        DebugLog("Altitude read unsuccessful", "main.py -> MissionStateUpdate()")
        return

    if missionMode == MISSION_MODES.PRELAUNCH:
        # Stage 1. The cansat lifts off higher than a predefined launch threshold

        DebugLog(f"PRELAUNCH: {altitudeData:.1f}m / {MISSION_LAUNCH_DELTATHRESHOLD:.1f}m",
                 "main.py -> MissionStateUpdate()")

        if altitudeData > MISSION_LAUNCH_DELTATHRESHOLD:
            missionMode = MISSION_MODES.LAUNCH
    elif missionMode == MISSION_MODES.LAUNCH:
        # Stage 2. The cansat has launched

        # Record highest altitude and evaluate if we are falling
        missionAltitudeMax = altitudeData if altitudeData > missionAltitudeMax else missionAltitudeMax

        if missionAltitudeMax - altitudeData < MISSION_LAUNCH_DELTATHRESHOLD:
            missionAltitudePrevious = altitudeData
            missionLandedTrigger = 0
            DebugLog(f"LAUNCH: {altitudeData - missionAltitudeMax:.1f}m / {-MISSION_LAUNCH_DELTATHRESHOLD}m",
                     "main.py -> MissionStateUpdate()")
            return

        # Check if we have stopped for an amount of iterations
        if abs(altitudeData - missionAltitudePrevious) < MISSION_LANDED_DELTATHRESHOLD:
            missionLandedTrigger += 1

            DebugLog(f"LANDING: {missionLandedTrigger} / {MISSION_LANDED_TRIGGER}", "main.py -> MissionStateUpdate()")

            if missionLandedTrigger >= MISSION_LANDED_TRIGGER:
                missionMode = MISSION_MODES.LANDED
        else:
            missionAltitudePrevious = altitudeData


# The heart of the Cansat
def MainCycle() -> None:
    global missionMode, missionAltitudeFailed

    previousTick: int = utime.ticks_ms()
    tickUpdateOffset: int = 0

    while True:
        utime.sleep(CANSAT_UPDATETIME - Clamp(tickUpdateOffset * 0.001, 0, CANSAT_UPDATETIME * 0.3))

        MissionStateUpdate()

        # MISSION STATUS: Cansat has been launched, run all systems nominally
        if missionMode != MISSION_MODES.PRELAUNCH:
            if not missionAltitudeFailed:
                ToggleSoilResistanceSensor(components.SoilResistanceServo, True)

            altitudeData, altitudeReadSuccess = GetAltitude(sensors.BMP)
            airTemperatureData, airTemperatureReadSucces = GetAirTemperature(sensors.BMP)
            airPressureData, airPressureReadSuccess = GetAirPressure(sensors.BMP)
            cansatPitch, cansatRoll = GetCansatPitchRoll(sensors.MPU)
            gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)
            airHumidityData, airHumidityReadSuccess = GetAirHumidity(sensors.DHT)
            soilRelativeHumidity: float = sensors.SoilResistance.MeasureResistance()

            components.CansatLogger.LogData(missionMode, gpsLatitude, gpsLongitude, cansatPitch, cansatRoll, airTemperatureData, altitudeData, airPressureData, airHumidityData, soilRelativeHumidity)

            components.Radio.Send(f"{missionMode}:{gpsLatitude}:{gpsLongitude}:{cansatPitch}:{cansatRoll}:{airTemperatureData}:{altitudeData}:{airPressureData}:{airHumidityData}:{soilRelativeHumidity}\n")

            DebugLog(f"{missionMode}:{gpsLatitude}:{gpsLongitude}:{cansatPitch}:{cansatRoll}:{airTemperatureData}:{altitudeData}:{airPressureData}:{airHumidityData}:{soilRelativeHumidity}", "main.py")

        # MISSION STATUS: Cansat has landed, start auditory and visual help cues for locating the cansat
        if missionMode == MISSION_MODES.LANDED:
            ToggleAlarmBuzzer(True)

            # Visible blinking
            for i in range(0, MISSION_LANDED_BLINKS):
                ToggleStatusLed(False)
                utime.sleep_ms(MISSION_LANDED_BLINKTIME)
                ToggleStatusLed(True)
                utime.sleep_ms(MISSION_LANDED_BLINKTIME)

            ToggleStatusLed(False)

        # Evaluate main loop tick differences
        currentTick: int = utime.ticks_ms()
        tickDifference: int = utime.ticks_diff(currentTick, previousTick)
        previousTick = currentTick
        tickUpdateOffset = tickDifference - 1000 if tickDifference > (1000 + tickUpdateOffset) else tickUpdateOffset


def Init():
    InitCansatBoot()  # First init sequence

    DebugLog("Initializing!", "main.py")

    # Standard initialization
    ToggleBuiltInLed(False)
    ToggleStatusLed(False)
    ToggleAlarmBuzzer(False)
    ToggleSoilResistanceSensor(components.SoilResistanceServo, False)

    sensors.BMP.use_case(BMP280_CASE_INDOOR)  # Is DROP an outdoor use case? :/

    # Core module initalization
    DebugLog("Initializing CansatCore.py!", "main.py")

    InitCansatCore(sensors.BMP, sensors.MPU)

    # Finalization
    DebugLog("Initialized!", "main.py")

    for i in range(0, INITALIZATION_BLINKS):
        utime.sleep_ms(INITALIZATION_BLINKTIME)
        ToggleStatusLed(True)
        utime.sleep_ms(INITALIZATION_BLINKTIME)
        ToggleStatusLed(False)

    ToggleStatusLed(False)

    DebugLog("Executing MainCycle()!\n", "main.py")

    MainCycle()


Init()


