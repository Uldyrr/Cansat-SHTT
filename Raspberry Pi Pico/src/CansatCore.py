from machine import Pin, ADC, UART
from imu import MPU6050
from bmp280 import BMP280
from dht import DHT11
from micropyGPS import MicropyGPS
from vector3d import Vector3d
from Servo import Servo
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
    PowerLed: Pin = Pin(2, Pin.OUT)


# // Constants
# -- General Cansat Constants
CANSAT_ADC16BIT: float = 2**16 - 1                  # 16-bit ADC
CANSAT_ADC12BIT: float = 2**12 - 1                  # 12-bit ADC
CANSAT_UPDATEHZ: float = 1.0                        # Hz
CANSAT_UPDATETIME: float = 1 / CANSAT_UPDATEHZ      # Seconds
CANSAT_ALTITUDECORRECTION: float = 120.0            # m | NOTE: Currently automatically updated in InitCansatCore() IF a BMP280 object is provided
CANSAT_SEALEVELPRESSURE: float = 1013.25            # hPa

# -- Initialization Cansat Constants
CANSAT_INITALIZATION_BLINKS: int = 5                # Count of power led blinks
CANSAT_INITALIZATION_BLINKTIME: int = int(100 / 2)  # Time of one power led blink

# -- Mission Constants
class MISSION_MODES:
    PRELAUNCH = 1,           # Hibernate mode, all systems will be off
    LAUNCH = 2,              # Mission mode, all systems will turned on
    LANDED = 3               # Retrival mode, all systems will continue running and an alarm buzzer will toggle

MISSION_LAUNCHALTITUDE: float = 3.0      # m
MISSION_LANDEDTHRESHOLD: float = 1.0     # m
MISSION_LANDEDTRIGGER: int = 10          # Count before we can consider the cansat landed

# -- Gass Constants
GAS_PPM_A: float = 116.6020682  # Parameter a as the coefficient for calculating PPM
GAS_PPM_B: float = 2.769034857  # Parameter b as the exponent for calculating PPM

# Parameters to model temperature and humidity depencdence
GAS_COR_A: float = 0.00035
GAS_COR_B: float = 0.02718
GAS_COR_C: float = 1.39538
GAS_COR_D: float = 0.0018

# -- Component Constants
MQ135_LOADRESISTANCE = 1.0  # kOhm
MQ131_LOADRESISTANCE = 1.0  # kOhm

BUZZER_MICROSECONDS: int = 1_000_000  # 1 / 1000 (ms) / 1000 (us)


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
    altitude: float = (((pressureRatio ** (1 / 5.257)) - 1) * airTemperatureKelvin) / 0.0065 - CANSAT_ALTITUDECORRECTION

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

    altitudeReadSuccess = airPressureSuccess and airTemperatureSuccess

    return altitudeData, altitudeReadSuccess


def GetAccelerationGyro(mpu: MPU6050, mpuData: dict = None) -> tuple[dict, dict, bool]:
    """
    Gets the current acceleration and gyro of the cansat using the MPU6050 IMU sensor

    Parameters
    ----------
    mpu : MPU6050
        The MPU6050 IMU sensor connected to the cansat
    mpuData : dict?
        A dictionary that when passed in will be updated with the mpu6050's data with the given keys: "Acceleration", "Gyroscope". Withh the values: "x", "y", "z"

    Returns
    -------
    tuple[dict, dict, bool]
        The acceleration and gyroscope data as dict objects and a value indicating whether the MPU6050 data was read
    """

    accelerationGyroSuccess: bool = True
    accelerationData: dict = {"x": 0, "y": 0, "z": 0}
    gyroData: dict = {"x": 0, "y": 0, "z": 0}

    try:
        accelerationData = {"x": mpu.accel.x, "y": mpu.accel.y, "z": mpu.accel.x}
        gyroData = {"x": mpu.gyro.x, "y": mpu.gyro.y, "z": mpu.gyro.x}
    except:
        accelerationGyroSuccess = False

    # Update the mpuData dict (IF PROVIDED) with data from the MPU6050
    if accelerationGyroSuccess and type(mpuData) is dict:
        mpuData["Acceleration"] = [
            accelerationData["x"],
            accelerationData["y"],
            accelerationData["z"]
        ]

        mpuData["Gyroscope"] = [
            gyroData["x"],
            gyroData["y"],
            gyroData["z"]
        ]

    return accelerationData, gyroData, accelerationGyroSuccess


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
    Uses a DHT11 object to get the current air humidity

    Note
    ----
    The air humidity is RELATIVE to the current air temperature

    Parameters
    ----------
    dht : DHT11
        An object of the DHT11 air humidity sensor class

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


def ToggleSoilMoistureSensor(soilMoistureServo: Servo, extendedState: bool) -> None:
    """
    EXTENDS or Retracts the POINTY soil moisture sensor

    Note
    ----
    THIS SENSOR HAS TWO FUCKING SHARP GROUND PENETRATORS

    Parameters
    ----------
    extendedState : bool
        The extented state of the soil moisture sensor. When set to true, EXERCISE CAUTION
    """
    if extendedState:
        soilMoistureServo.move(90)
    else:
        soilMoistureServo.move(0)


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
    sleepTime: int = int(BUZZER_MICROSECONDS / pitch / 2)  # Microseconds pitch time
    loopIntervals: int = int(time * BUZZER_MICROSECONDS / sleepTime)

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


def TogglePowerLed(ledState: bool = None) -> None:
    """
    Toggles the cansat's power led to show whether or not the cansat is powered

    Parameters
    ----------
    ledState : bool
        The on state of the power led. When None, the function will inverse the led state
    """

    if ledState is None:
        ledState = not _components.PowerLed.value()

    _components.PowerLed.value(ledState)


# Init
def InitCansatCore(bmp: BMP280 = None) -> None:
    """
    Initializes the Core module of SHTT Cansat

    Parameters
    ----------
    bmp : BMP280?
        The BMP280 air pressure & temperature sensor object, or None if no automatic altitude correction should be performed
    """

    global CANSAT_ALTITUDECORRECTION

    # Initialize constants
    if bmp is not None:
        CANSAT_ALTITUDECORRECTION = 0  # Set to zero to get the actual altitude offset one will get from calling GetAltitude()
        CANSAT_ALTITUDECORRECTION, _ = GetAltitude(bmp)








