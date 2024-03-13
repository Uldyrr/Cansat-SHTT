from CansatCore import *
from CansatCommunication import RadioCom
from CansatLogger import CansatLogger
from CansatGas import GasSensor, GASSENSOR_CALIBRATIONGAS, GASSENSOR_RZERO
from machine import Pin, ADC, UART, I2C
from imu import MPU6050
from bmp280 import *
from dht import DHT11
from micropyGPS import MicropyGPS
from Servo import Servo
import _thread
import utime


# Values
# // Sensors
class sensors:
    MPU: MPU6050 = MPU6050(I2C(0, sda=Pin(20), scl=Pin(21)))
    BMP: BMP280 = BMP280(I2C(0, sda=Pin(20), scl=Pin(21)))
    DHT: DHT11 = DHT11(Pin(9))
    MQ135: GasSensor = GasSensor(27, 1.0, GASSENSOR_RZERO.OXYGEN, GASSENSOR_CALIBRATIONGAS.OXYGEN)
    MQ131: GasSensor = GasSensor(26, 1.0, GASSENSOR_RZERO.OZONE, GASSENSOR_CALIBRATIONGAS.OZONE)


# // Components
class components:
    SoilMoistureServo: Servo = Servo(8)
    GPSSerialBus: UART = UART(1, 9600, tx=Pin(4), rx=Pin(5))
    GPS: MicropyGPS = MicropyGPS(location_formatting='dd')
    Radio: RadioCom = RadioCom(UART(0, 9600, tx=Pin(16), rx=Pin(17)))
    CansatLogger = CansatLogger()


# // Sensor data
mpuData: dict = {}

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

        if abs(altitudeData - missionPreviousAltitude) < MISSION_LANDEDTHRESHOLD:  # Increment trigger if the delta altitude is below a delta thershold
            missionPreviousAltitudeTrigger += 1
        else:  # If our altitude change is too high, reset values and continue trying to evaluate whether we've landed
            missionPreviousAltitudeTrigger = 0
            missionPreviousAltitude = altitudeData

        missionMode = MISSION_MODES.LANDED if missionPreviousAltitudeTrigger >= MISSION_LANDEDTRIGGER else MISSION_MODES.LAUNCH


# The heart of the Cansat
def MainCycle() -> None:
    global missionMode, missionAltitudeFailed, missionPreviousAltitude, missionPreviousAltitudeTrigger

    previousTick: int = utime.ticks_ms()
    tickUpdateOffset: int = 0

    while True:
        utime.sleep(CANSAT_UPDATETIME - Clamp(tickUpdateOffset * 0.001, 0, CANSAT_UPDATETIME * 0.3))

        MissionStateUpdate()

        if missionMode == MISSION_MODES.PRELAUNCH:
            print(f"AWAITING PROPER HEIGHT! Previous Altitude: {missionPreviousAltitude:.2f} | Launch Altitude: {MISSION_LAUNCHALTITUDE:.2f}")

        # MISSION STATUS: Cansat has been launched, run all systems nominally
        if missionMode != MISSION_MODES.PRELAUNCH:
            ToggleSoilMoistureSensor(components.SoilMoistureServo, True)

            altitudeData, altitudeReadSuccess = GetAltitude(sensors.BMP)
            airTemperatureData, airTemperatureReadSucces = GetAirTemperature(sensors.BMP)
            airPressureData, airPressureReadSuccess = GetAirPressure(sensors.BMP)
            gpsLatitude, gpsLongitude = GetGPSLatitudeLongitude(components.GPS, components.GPSSerialBus)
            airHumidityData, airHumidityReadSuccess = GetAirHumidity(sensors.DHT)

            # components.CansatLogger.LogData(gpsLatitude, gpsLongitude, airTemperatureData, airPressureData)

            # components.Radio.Send(f"{GetBuiltInTemperature()}:{airHumidityData}\n")

            print(f"Mission Code: {missionMode} | Alt: {altitudeData:.2f} | Air Temp: {airTemperatureData:.1f} | Air Pa: {airPressureData:.1f} | LatLng: {gpsLatitude}, {gpsLongitude} | Landing: ({abs(altitudeData - missionPreviousAltitude)} | {missionPreviousAltitudeTrigger}/{MISSION_LANDEDTRIGGER})")

        # MISSION STATUS: Cansat has landed, continue systems running, but start the alarm buzzer
        if missionMode == MISSION_MODES.LANDED:  # Amazing use of power
            ToggleAlarmBuzzer(True)
            TogglePowerLed(True)
            ToggleBuiltInLed(True)

        # Evaluate tick differences
        currentTick = utime.ticks_ms()
        tickDifference = currentTick - previousTick
        previousTick = currentTick
        tickUpdateOffset = tickDifference - 1000 if tickDifference > (1000 + tickUpdateOffset) else tickUpdateOffset


def Init():
    print("Initializing!")

    # Standard initialization
    TogglePowerLed(False)
    ToggleAlarmBuzzer(False)
    ToggleSoilMoistureSensor(components.SoilMoistureServo, False)

    sensors.BMP.use_case(BMP280_CASE_INDOOR)  # Is DROP an outdoor use case? :/

    # Core module initalization
    print("Initializing CansatCore.py!")

    InitCansatCore(sensors.BMP)

    # Finalization
    print("Initialized!")

    for i in range(0, CANSAT_INITALIZATION_BLINKS):
        utime.sleep_ms(CANSAT_INITALIZATION_BLINKTIME)
        TogglePowerLed(True)
        utime.sleep_ms(CANSAT_INITALIZATION_BLINKTIME)
        TogglePowerLed(False)

    TogglePowerLed(False)

    print("Starting MainCycle()!\n")

    MainCycle()


Init()



