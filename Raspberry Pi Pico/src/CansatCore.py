from machine import Pin, ADC, UART
from imu import MPU6050
from bmp280 import BMP280
from dht import DHT11
from micropyGPS import MicropyGPS
from vector3d import Vector3d
import _thread
import utime

# Module values
# // Values
_alarmBuzzerRunning: bool = False  # Used in relation with the cansat's alarm buzzer


# // Sensors
class _sensors:
    BuiltInTemperatureSensor: ADC = ADC(4)


# // Components
class _components:
    BuiltInLed: Pin = Pin(25, Pin.OUT)
    AlarmBuzzer: Pin = Pin(20, Pin.OUT)


# // Constants
CANSAT_UPDATEHZ: float = 1.0  # hz
CANSAT_ALTITUDECORRECTION: float = 120.0  # m, NOTE: Currently automatically updated in InitCansatCore() if a BMP280 object is provided
CANSAT_SEALEVELPRESSURE: float = 1013.25  # hPa

BUZZER_ALARMHZ: int = 800  # The frequency at which the buzzer will play
BUZZER_MICROSECONDS: int = 1_000_000  # 1 / 1000 (ms) / 1000 (us)
BUZZER_ALARMSLEEPTIME: int = int(
    BUZZER_MICROSECONDS / BUZZER_ALARMHZ / 2)  # The time to yield every alarm buzzer value() change to get the specified frequency of sound

HIGH: bool = True
LOW: bool = False


# Primary mission helper functions
def GetBuiltInTemperature() -> float:
    """
    Gets the current ambient temperature using the Raspberry Pi Pico's built-in temperature sensor

    :return: A float of the current temperature using the Raspberry Pi Pico's built-in temperature sensor
    """

    adcValue: float = _sensors.BuiltInTemperatureSensor.read_u16()
    voltage: float = (3.3 / 65535) * adcValue
    temperature: float = 27 - (voltage - 0.706) / 0.001721

    return round(temperature, 2)


def GetHypsometricEquationAltitude(airPressurehPa: float, airTemperatureKelvin: float) -> float:
    """
    Gets the current altitude of the cansat using the Hypsometric equation

    Note
    ----
    float
        Max height above sea level: ~11km

    Parameters
    ----------
    airPressurehPa : float
        The air pressure in hectopascals

    airTemperatureKelvin : float
        The air temperature in kelvin

    Returns
    -------
    float
        A float of the current altitude of the cansat based on the given air pressure and temperature
    """

    pressureRatio: float = CANSAT_SEALEVELPRESSURE / airPressurehPa
    altitude: float = (((pressureRatio ** (1 / 5.257)) - 1) * airTemperatureKelvin) / 0.0065 - CANSAT_ALTITUDECORRECTION

    return altitude


def GetAirTemperature(bmp: BMP280) -> float:
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
    """

    return bmp.temperature  # Getting the air temperature using the bmp280 as shown here, is very easy lol


def GetAirPressure(bmp: BMP280) -> float:
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
    """

    return bmp.pressure


def GetAltitude(bmp: BMP280) -> float:
    """
    Gets the current pressure and temperature using bmp280 to calculate the cansat's current altitude

    Parameters
    ----------
    bmp : BMP280
        The bmp280 air pressure & temperature sensor connected to the cansat

    Returns
    -------
    float
        The current altitude of the cansat in meters
    """

    airPressurehPa: float = GetAirPressure(bmp) * 0.01  # hPa
    airTemperatureKelvin: float = GetAirTemperature(bmp) + 273.15  # Kelvin

    altitudeData: float = GetHypsometricEquationAltitude(airPressurehPa, airTemperatureKelvin)

    return altitudeData


def GetAccelerationGyro(mpu: MPU6050, mpuData: dict = None) -> tuple[Vector3d, Vector3d]:
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
    tuple[Vector3d, Vector3d]
        The acceleration and gyroscope data as Vector3d objects
    """

    accelerationData: Vector3d = mpu.accel
    gyroData: Vector3d = mpu.gyro

    # Update the mpuData dict (IF PROVIDED) with data from the MPU6050
    if type(mpuData) is dict:
        mpuData["Acceleration"] = [
            accelerationData.x,
            accelerationData.y,
            accelerationData.z
        ]

        mpuData["Gyroscope"] = [
            gyroData.x,
            gyroData.y,
            gyroData.z
        ]

    return accelerationData, gyroData


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
def GetAirHumidity(dht: DHT11) -> float:
    """
    Uses a DHT11 object to get the current air humidity

    Note
    ----
    The air humidity is RELATIVE to the current air temperature

    Parameters
    ----------
    dht : DHT11
        An object of the DHT11 air humidity sensor class
    """
    dht.measure()

    return float(f"{dht.humidity():.2f}")


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


def _AlarmBuzzerUpdate() -> None:
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
        _components.AlarmBuzzer.value(not _components.AlarmBuzzer.value())
        utime.sleep_us(BUZZER_ALARMSLEEPTIME)


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

        _thread.start_new_thread(_AlarmBuzzerUpdate, ())
    elif alarmState is False and _alarmBuzzerRunning:
        _alarmBuzzerRunning = False


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
        CANSAT_ALTITUDECORRECTION = GetAltitude(bmp)
