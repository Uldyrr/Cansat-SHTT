from CansatDataStructures import Vector3
from machine import Pin, ADC, UART
from imu import MPU6050
from bmp280 import BMP280
from dht import DHT11
from micropyGPS import MicropyGPS
from Servo import Servo
from math import atan2, sqrt, pi
import _thread
import utime
import random

# Module values
# // Values
_alarmBuzzerRunning: bool = False  # Used in relation with the cansat's alarm buzzer


# // Sensors
class _sensors:
    BuiltInTemperatureSensor: ADC = ADC(4)


# // Components
class _components:
    BuiltInLed: Pin = Pin(25, Pin.OUT)
    AlarmBuzzer: Pin = Pin(13, Pin.OUT)
    StatusLed: Pin = Pin(2, Pin.OUT)


# // Constants
# -- General Cansat Constants
CANSAT_UPDATEHZ: float = 1.0                               # Hz
CANSAT_UPDATETIME: float = 1 / CANSAT_UPDATEHZ             # Seconds
CANSAT_BOOTTIME: int = utime.ticks_ms()                    # Ms

CANSAT_ADC16BIT: float = 2**16 - 1                         # 16-bit ADC
CANSAT_ADC12BIT: float = 2**12 - 1                         # 12-bit ADC
CANSAT_RAD2DEG: float = 180.0 / pi                         # Ratio to calculate degrees from radians
CANSAT_MICROSECONDS: int = 1_000_000                       # 1 / 1000 (ms) / 1000 (us)
CANSAT_SEALEVELPRESSURE: float = 1013.25                   # hPa

CANSAT_CORRECTION_ALTITUDE: float = 0.0                   # m | NOTE: Currently automatically updated in InitCansatCore() IF a BMP280 object is provided
CANSAT_CORRECTION_ACCELEROMETER: Vector3 = Vector3.Empty()  # x, y, z correction

# -- Initialization Constants
INITALIZATION_ACCELEROMETER_MEASUREMENTS: int = 50  # Measurement calibration count

INITALIZATION_BLINKS: int = 5                        # Count of power led blinks
INITALIZATION_BLINKTIME: int = int(100 / 2)          # ms, time of one power led blink

# -- Mission Constants
class MISSION_MODES:
    PRELAUNCH = 1,           # Hibernate mode, all systems will be off
    LAUNCH = 2,              # Mission mode, all systems will turned on
    LANDED = 3,              # Retrival mode, all systems will continue running and an alarm buzzer will toggle

MISSION_LAUNCH_ALTITUDE: float = 0.3          # m
MISSION_LAUNCH_THRESHOLD: float = 0.3         # m
MISSION_LANDED_THRESHOLD: float = 1.0         # m
MISSION_LANDED_TRIGGER: int = 10              # Count before we can consider the cansat landed
MISSION_LANDED_BLINKS: int = 10               # Count of power led blinks
MISSION_LANDED_BLINKTIME: int = int(100 / 2)  # ms, time of one power led blink


# Generic helper functions
def Clamp(value: any, minValue: any = None, maxValue: any = None) -> any:
    if type(minValue) is not None and value < minValue:
        return minValue
    elif type(maxValue) is not None and value > maxValue:
        return maxValue

    return value


# Primary mission helper functions
def GetBuiltInTemperature() -> float:
    """
    Gets the current ambient temperature using the Raspberry Pi Pico's built-in temperature sensor

    :return: A float of the current temperature using the Raspberry Pi Pico's built-in temperature sensor
    """

    adcValue: float = _sensors.BuiltInTemperatureSensor.read_u16()
    voltage: float = (3.3 / CANSAT_ADC16BIT) * adcValue
    temperature: float = 27 - (voltage - 0.706) / 0.001721

    return round(temperature, 2)


def GetHypsometricEquationAltitude(airPressurePa: float, airTemperatureCelsius: float) -> float:
    """
    Gets the current altitude of the cansat using the Hypsometric equation

    Note
    ----
    float
        Max height above sea level: ~11km

    Parameters
    ----------
    airPressurePa : float
        The air pressure in pascals

    airTemperatureCelsius : float
        The air temperature in celsius

    Returns
    -------
    float
        A float of the current altitude of the cansat based on the given air pressure and temperature
    """

    airPressurehPa = airPressurePa * 0.01
    airTemperatureKelvin = airTemperatureCelsius + 273.15

    pressureRatio: float = CANSAT_SEALEVELPRESSURE / airPressurehPa
    altitude: float = (((pressureRatio ** (1 / 5.257)) - 1) * airTemperatureKelvin) / 0.0065 - CANSAT_CORRECTION_ALTITUDE

    return altitude


def GetAirTemperature(bmp: BMP280) -> tuple[float, bool]:
    """
    Uses the bmp280 air pressure & temperature sensor to get the current air temperature

    Parameters
    ----------
    bmp : BMP280
        The BMP280 air pressure & temperature sensor object

    Returns
    -------
    float
        A float air temperature value
    bool
        A boolean value indicating whether the BMP280's temperature value was read successfully
    """

    try:
        return bmp.temperature, True
    except:
        return -1.0, False


def GetAirPressure(bmp: BMP280) -> tuple[float, bool]:
    """
    Uses the bmp280 air pressure & temperature sensor to get the current air pressure

    Parameters
    ----------
    bmp : BMP280
        The BMP280 air pressure & temperature sensor object

    Returns
    -------
    float
        A float air pressure value
    bool
        A boolean value indicating whether the BMP280's pressure value was read successfully
    """

    try:
        return bmp.pressure, True
    except:
        return -1.0, False


def GetAltitude(bmp: BMP280) -> tuple[float, bool]:
    """
    Gets the current pressure and temperature using bmp280 to calculate the cansat's current altitude

    Parameters
    ----------
    bmp : BMP280
        The bmp280 air pressure & temperature sensor connected to the cansat

    Returns
    -------
    tuple[float, bool]
        The current altitude of the cansat in meters and a boolean value indicating that all sensor values were read successfully
    """

    airPressurePa, airPressureSuccess = GetAirPressure(bmp)
    airTemperatureCelsius, airTemperatureSuccess = GetAirTemperature(bmp)

    altitudeData: float = GetHypsometricEquationAltitude(airPressurePa, airTemperatureCelsius)
    altitudeReadSuccess: bool = airPressureSuccess and airTemperatureSuccess

    return altitudeData, altitudeReadSuccess


def GetAccelerationGyro(mpu: MPU6050) -> tuple[Vector3, Vector3, bool]:
    """
    Gets the current acceleration and gyro of the cansat using the MPU6050 IMU sensor

    Parameters
    ----------
    mpu : MPU6050
        The MPU6050 IMU sensor connected to the cansat

    Returns
    -------
    tuple[Vector3d?, Vector3d?, bool]
        The acceleration and gyroscope data as Vector3d references and a boolean value indicating whether the MPU6050 data was read
    """

    accelerationGyroSuccess: bool = True
    accelerationData: Vector3 = Vector3.Empty()
    gyroData: Vector3 = Vector3.Empty()

    try:
        accelerationData = Vector3(mpu.accel.x - CANSAT_CORRECTION_ACCELEROMETER.X, mpu.accel.y - CANSAT_CORRECTION_ACCELEROMETER.Y, mpu.accel.z - CANSAT_CORRECTION_ACCELEROMETER.Z)
        gyroData = Vector3(mpu.gyro.x, mpu.gyro.y, mpu.gyro.z)
    except:
        accelerationGyroSuccess = False

    return accelerationData, gyroData, accelerationGyroSuccess


def GetCansatPitchRoll(mpu: MPU6050) -> tuple[float, float]:
    accelerationData, gyroData, accelerationGyroSuccess = GetAccelerationGyro(mpu)
    cansatPitch: float = 0.0
    cansatRoll: float = 0.0

    # Corrected for initial upward orientation (Up vector: Y axis)
    if accelerationGyroSuccess:
        cansatPitch = atan2(accelerationData.Z, sqrt(accelerationData.Y * accelerationData.Y + accelerationData.X * accelerationData.X)) * CANSAT_RAD2DEG
        cansatRoll = atan2(-accelerationData.X, sqrt(accelerationData.Z * accelerationData.Z + accelerationData.Y * accelerationData.Y)) * CANSAT_RAD2DEG

    return cansatPitch, cansatRoll


def GetGPSLatitudeLongitude(gps: MicropyGPS, gpsSerialBus: UART) -> tuple[list, list]:
    """
    Uses the cansat's gps module to get the current latitude and longitude of the cansat

    Parameters
    ----------
    gps : MicropyGPS
        The MicropyGPS object of the gps module component (Neo-7m) connected to the cansat
    gpsSerialBus : UART
        The UART Serial Bus Communication object to use with reading the gps data

    Returns
    -------
    tuple[list, list]
        The latitude and longitude data from the gps
    """

    gpsMessage: any = gpsSerialBus.readline()

    # Update the MicropyGPS object with data from the gps module component
    if gpsMessage:
        for char in gpsMessage:
            gps.update(chr(char))

    return gps.latitude, gps.longitude


# Secondary mission helper functions
def GetAirHumidity(dht: DHT11) -> tuple[float, bool]:
    """
    Uses a DHT11 object to get the current air relative humidity

    Note
    ----
    The air humidity is RELATIVE to the current air temperature

    Parameters
    ----------
    dht : DHT11
        An object of the DHT11 air relative humidity sensor class

    Returns
    -------
    float
        The relative air humidity floating point value
    bool
        A boolean value indicating whether the DHT11's air humidity value was read successfully
    """

    try:
        dht.measure()

        return round(dht.humidity(), 2), True
    except:
        return -1.0, False


def ToggleSoilResistanceSensor(soilResistanceServo: Servo, extendedState: bool) -> None:
    """
    EXTENDS or Retracts the POINTY soil resistance sensor

    Note
    ----
    THIS SENSOR HAS TWO FUCKING SHARP GROUND PENETRATORS

    Parameters
    ----------
    soilResistanceServo : Servo
        The micro servo to use with extending or retracting the soil resistance sensor
    extendedState : bool
        The extented state of the soil resistance sensor. When set to true, EXERCISE CAUTION
    """
    if extendedState:
        soilResistanceServo.move(90)
    else:
        soilResistanceServo.move(0)


# Helper component functions
def ToggleBuiltInLed(ledState: bool = None) -> None:
    """
    Toggles the Raspberry Pi Pico's built-in LED on or off

    Parameters
    ----------
    ledState : bool
        A boolean value expressing the power state of the built-in LED. Null if the power state should be toggled
    """

    # Inverse the current power state of the built-in LED
    if ledState is None:
        ledState = not _components.BuiltInLed.value()

    _components.BuiltInLed.value(ledState)


def _AlarmBuzzerIntervalUpdate(pitch: int, time: float) -> None:
    """
    Used to create amazing alarm buzzer sounds

    Parameters
    ----------
    pitch : int
        An integer value that will be used to create a specific pitch of sound
    time : float
        A floating point type value that will be used to play the pitch for a certain amount of time
    """
    sleepTime: int = int(CANSAT_MICROSECONDS / pitch / 2)  # Microseconds pitch time
    loopIntervals: int = int(time * CANSAT_MICROSECONDS / sleepTime)

    for _ in range(0, loopIntervals):
        _components.AlarmBuzzer.value(1)
        utime.sleep_us(sleepTime)
        _components.AlarmBuzzer.value(0)
        utime.sleep_us(sleepTime)


def _AlarmBuzzerThreadUpdate() -> None:
    """
    Toggles the power of the alarm buzzer in a fashion that creates a sound with a particular frequency

    Note
    ----
    The function's while loop update will stop when the alarm buzzer's running state is set to False

    See Also
    --------
    ToggleAlarmBuzzer(alarmState: bool)
    """

    global _alarmBuzzerRunning

    while _alarmBuzzerRunning:
        _AlarmBuzzerIntervalUpdate(random.randint(100, 3000), random.randint(5, 10) / 100)

    _thread.exit()


def ToggleAlarmBuzzer(alarmState: bool = None) -> None:
    """
    Toggles the running state of the cansat's alarm buzzer to a given state

    Note
    ----
    This function will create a new thread to operate on when the alarm state is true!

    Parameters
    ----------
    alarmState : bool
        A boolean value expressing the running state of the cansat's alarm buzzer. When None, the function will simply inverse the current running state
    """

    global _alarmBuzzerRunning

    # Inverse the current running state of the alarm buzzer
    if alarmState is None:
        alarmState = not _alarmBuzzerRunning

    if alarmState and _alarmBuzzerRunning is False:
        _alarmBuzzerRunning = True

        _thread.start_new_thread(_AlarmBuzzerThreadUpdate, ())
    elif alarmState is False and _alarmBuzzerRunning:
        _alarmBuzzerRunning = False


def ToggleStatusLed(ledState: bool = None) -> None:
    """
    Toggles the cansat's status led to show that the cansat is functioning

    Parameters
    ----------
    ledState : bool
        The boolean on state of the status led. When None, the function will inverse the led state
    """

    if ledState is None:
        ledState = not _components.StatusLed.value()

    _components.StatusLed.value(ledState)


# Generic function for printing debug/log messages
def DebugLog(message: str, source: str = None) -> None:
    """
    Debug/Log generic message printing function

    Parameters
    ----------
    message : str
        The message to print
    source : str?
        The source .py file. Performs a regular print when not provided
    """

    if source:
        print(f"[{utime.ticks_diff(utime.ticks_ms(), CANSAT_BOOTTIME) / 1000.0:.3f}s | {source}]: {message}")
    else:
        print(message)


# Init
def InitCansatCore(bmp: BMP280 = None, mpu: MPU6050 = None) -> None:
    """
    Initializes the Core module of SHTT Cansat

    Parameters
    ----------
    bmp : BMP280?
        The BMP280 air pressure & temperature sensor object, or None if no automatic altitude correction should be performed
    mpu : MPU6050?
        The MPU6050 IMU sensor object, or None if no automatic calibration should be performed
    """

    global CANSAT_CORRECTION_ALTITUDE, CANSAT_CORRECTION_ACCELEROMETER

    # Initialize sensor constants
    if bmp is not None:
        DebugLog("Calibrating BMP altitude", "CansatCore.py")

        CANSAT_CORRECTION_ALTITUDE, _ = GetAltitude(bmp)

        DebugLog(f"Got altitude correction: {CANSAT_CORRECTION_ALTITUDE:.2f}m", "CansatCore.py")

    if mpu is not None:
        DebugLog("Calibrating MPU6050", "CansatCore.py")

        t = utime.ticks_ms()

        accelerometerTotal = Vector3.Empty()

        for i in range(0, INITALIZATION_ACCELEROMETER_MEASUREMENTS):
            accelerometerData, gyroData, _ = GetAccelerationGyro(mpu)

            accelerometerTotal.X += accelerometerData.X
            accelerometerTotal.Y += accelerometerData.Y
            accelerometerTotal.Z += accelerometerData.Z

        CANSAT_CORRECTION_ACCELEROMETER.X = accelerometerTotal.X / INITALIZATION_ACCELEROMETER_MEASUREMENTS
        CANSAT_CORRECTION_ACCELEROMETER.Y = accelerometerTotal.Y / INITALIZATION_ACCELEROMETER_MEASUREMENTS - 1.0  # Y will be in the direction of gravity
        CANSAT_CORRECTION_ACCELEROMETER.Z = accelerometerTotal.Z / INITALIZATION_ACCELEROMETER_MEASUREMENTS

        DebugLog(f"Got accelerometer correction: [{CANSAT_CORRECTION_ACCELEROMETER}] in {utime.ticks_diff(utime.ticks_ms(), t)}ms", "CansatCore.py")
