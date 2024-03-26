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
    MPU: MPU6050 = MPU6050(I2C(0, sda=Pin(20), scl=Pin(21)))
    BMP: BMP280 = BMP280(I2C(0, sda=Pin(20), scl=Pin(21)))
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
missionPreviousAltitude: float = 0.0
missionPreviousAltitudeTrigger: float = 0.0


# The mission state status update
def MissionStateUpdate() -> None:
    global missionMode, missionAltitudeFailed, missionPreviousAltitude, missionPreviousAltitudeTrigger

    altitudeData, altitudeReadSuccess = GetAltitude(sensors.BMP)

    # Stop the cansat from going to MISSION_MODES.LANDED and inform the ground station if the bmp280 has gone kaput
    if not altitudeReadSuccess:
        missionAltitudeFailed = True

    if missionMode == MISSION_MODES.PRELAUNCH:
        # Check whether the cansat is above a certain valid launch altitude to confirm launch
        missionMode = MISSION_MODES.LAUNCH if altitudeData >= MISSION_LAUNCHALTITUDE else MISSION_MODES.PRELAUNCH
        missionPreviousAltitude = altitudeData
    elif missionMode == MISSION_MODES.LAUNCH and not missionAltitudeFailed:
        # Check whether the cansat is below the launch alitude and stays under an altitude for a certain amount of time
        if altitudeData >= MISSION_LAUNCHALTITUDE:  # If we are above the launch altitude, reset values & early return
            missionPreviousAltitude = altitudeData
            missionPreviousAltitudeTrigger = 0
            return

        if abs(altitudeData - missionPreviousAltitude) < MISSION_LANDED_THRESHOLD:  # Increment trigger if the delta altitude is below a delta thershold
            missionPreviousAltitudeTrigger += 1
        else:  # If our altitude change is too high, reset values and continue trying to evaluate whether we've landed
            missionPreviousAltitudeTrigger = 0
            missionPreviousAltitude = altitudeData

        missionMode = MISSION_MODES.LANDED if missionPreviousAltitudeTrigger >= MISSION_LANDED_TRIGGER else MISSION_MODES.LAUNCH


# The heart of the Cansat
def MainCycle() -> None:
    global missionMode, missionAltitudeFailed, missionPreviousAltitude, missionPreviousAltitudeTrigger

    previousTick: int = utime.ticks_ms()
    tickUpdateOffset: int = 0

    while True:
        utime.sleep(CANSAT_UPDATETIME - Clamp(tickUpdateOffset * 0.001, 0, CANSAT_UPDATETIME * 0.3))

        MissionStateUpdate()

        if missionMode == MISSION_MODES.PRELAUNCH:
            DebugLog(f"AWAITING PROPER HEIGHT! Previous Altitude: {missionPreviousAltitude:.2f} | Launch Altitude: {MISSION_LAUNCHALTITUDE:.2f}", "main.py")

        # MISSION STATUS: Cansat has been launched, run all systems nominally
        if missionMode != MISSION_MODES.PRELAUNCH:
            ToggleSoilResistanceSensor(components.SoilResistanceServo, True)

            altitudeData, altitudeReadSuccess = GetAltitude(sensors.BMP)
            airTemperatureData, airTemperatureReadSucces = GetAirTemperature(sensors.BMP)
            airPressureData, airPressureReadSuccess = GetAirPressure(sensors.BMP)
            cansatPitch, cansatRoll = GetCansatPitchRoll(sensors.MPU)
            gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)
            airHumidityData, airHumidityReadSuccess = GetAirHumidity(sensors.DHT)
            soilRelativeHumidity: float = sensors.SoilResistance.MeasureResistance()

            # components.CansatLogger.LogData(gpsLatitude, gpsLongitude, airTemperatureData, airPressureData)

            # components.Radio.Send(f"{GetBuiltInTemperature()}:{airHumidityData}\n")

            DebugLog(f"Mission Code: {missionMode} | Alt: {altitudeData:.2f} | Air Temp: {airTemperatureData:.1f} | Air Pa: {airPressureData:.1f} | LatLng: {gpsLatitude}, {gpsLongitude} | Landing: ({abs(altitudeData - missionPreviousAltitude)} | {missionPreviousAltitudeTrigger}/{MISSION_LANDED_TRIGGER})", "main.py")

        # MISSION STATUS: Cansat has landed, continue systems running, but start auditory and visual help cues for locating the cansat
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
