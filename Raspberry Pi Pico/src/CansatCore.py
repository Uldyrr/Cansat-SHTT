from machine import Pin, ADC, UART
from imu import MPU6050
from bmp280 import BMP280
from micropyGPS import MicropyGPS
from lora_e32 import LoRaE32
from lora_e32_operation_constant import ResponseStatusCode
from vector3d import Vector3d
import utime
import _thread


# Module values
# // Values
__alarmBuzzerRunning: bool = False  # Used in relation with the cansat's alarm buzzer

# // Sensors
__builtInTemperatureSensor: ADC = ADC(4)

# // Components
__builtInLed: Pin = Pin(25, Pin.OUT)
__alarmBuzzer: Pin = Pin(20, Pin.OUT)

# // Constants
CANSAT_UPDATEHZ: float = 1  # Hz
CANSAT_ALTITUDECORRECTION: float = 120.0  # M
CANSAT_SEALEVELPRESSURE: float = 1013.25  # hPa

LORA_ADDRESS: int = 0x5  # An int8 (byte) address of the reciever that is supposed to acquire the message in Fixedmode
LORA_CHANNEL: int = 0x17  # The channel (Frequency) at which to send the message at

BUZZER_ALARM_HZ: int = 800  # The frequency at which the buzzer will play
BUZZER_MICROSECONDS: int = 1000000  # 1 / 1000 (ms) / 1000 (us)

HIGH: bool = True
LOW: bool = False


def GetBuiltInTemperature() -> float:
    """
    Gets the current ambient temperature using the Raspberry Pi Pico's built-in temperature sensor

    :return: A float of the current temperature using the Raspberry Pi Pico's built-in temperature sensor
    """

    adcValue: float = __builtInTemperatureSensor.read_u16()
    voltage: float = (3.3 / 65535) * adcValue
    temperature: float = 27 - (voltage - 0.706) / 0.001721

    return round(temperature, 2)


def GetHypsometricAltitude(airPressurehPa: float, airTemperatureKelvin: float) -> float:
    """
    Gets the current altitude using the Hypsometric equation

    .. note:: Max height above sea level: ~11km

    :param float airPressurehPa: The air pressure in hectopascals
    :param float airTemperatureKelvin: The air temperature in kelvin
    :return: A float of the current altitude of the cansat based on the given air pressure and temperature
    """

    pressureRatio: float = CANSAT_SEALEVELPRESSURE / airPressurehPa
    altitude: float = (((pressureRatio ** (1 / 5.257)) - 1) * airTemperatureKelvin) / 0.0065 - CANSAT_ALTITUDECORRECTION

    return altitude


def GetMPUAccelerationGyroTemp(mpu: MPU6050, bmp: BMP280, mpuData: dict = None) -> tuple[Vector3d, Vector3d, float]:
    """
    Gets the current acceleration, gyro, and temperature data using MPU6050 and BMP280

    :param MPU6050 mpu: The mpu6050 sensor connected to the cansat
    :param BMP280 bmp: The bmp280 sensor connected to the cansat
    :param dict? mpuData: A dictionary that when passed will be updated with both the mpu's and bmp's data with the given keys: "Acceleration", "Gyroscope", "Temperature"
    :return: The acceleration, gyroscope, and air temperature data
    """

    accelerationData: Vector3d = mpu.accel
    gyroData: Vector3d = mpu.gyro
    airTemperatureData: float = bmp.temperature

    # Update the mpuData dict (IF PROVIDED) with data from the MPU6050
    if type(mpuData) is dict:
        mpuData["Acceleration"] = {
            "x": accelerationData.x,
            "y": accelerationData.y,
            "z": accelerationData.z
        }

        mpuData["Gyroscope"] = {
            "x": gyroData.x,
            "y": gyroData.y,
            "z": gyroData.z
        }

        mpuData["Temperature"] = airTemperatureData

    return accelerationData, gyroData, airTemperatureData


def GetBMPPressureAltitude(bmp: BMP280, airTemperature: float) -> tuple[float, float]:
    """
    Gets the current pressure and uses it to calculate the cansat's current altitude

    :param BMP280 bmp: The bmp280 sensor connected to the cansat
    :param float airTemperature: The air temperature in Celsius measured by the cansat
    :return: The air pressure measured in Pascals and the current altitude of the cansat
    """

    airPressureData: float = bmp.pressure  # Pa

    hpaAirPressure: float = airPressureData * 0.01  # hPa
    airTemperatureKelvin: float = airTemperature + 273.15  # Kelvin

    altitudeData: float = GetHypsometricAltitude(hpaAirPressure, airTemperatureKelvin)

    return airPressureData, altitudeData


def GetGPSLatitudeLongitude(gps: MicropyGPS, gpsSerialBus: UART) -> tuple[list, list]:
    """
    Uses the cansat's gps module to get the current latitude and longitude of the cansat

    :param MicropyGPS gps: The MicropyGPS object of the gps module component (Neo-7m) connected to the cansat
    :param UART gpsSerialBus: The UART Serial Bus Communication object to use with reading the gps data
    :return: The latitude and longitude data from the gps
    """

    gpsMessage: any = gpsSerialBus.readline()

    # Update the MicropyGPS object with data from the gps module component
    if gpsMessage:
        for char in gpsMessage:
            gps.update(chr(char))

    return gps.latitude, gps.longitude


def LoRaSendDictData(loRa: LoRaE32, dictData: dict) -> str:
    """
    Sends a dictionary through radio using a long range (LoRa) radio component in Fixedmode

    :param LoRaE32 loRa: The long range (LoRa) radio component to use with transmitting the data
    :param dict dictData: The dictionary data to send over to a long range (LoRa) radio component reciever
    :return: The string ResponseStatusCode enum code
    """

    return ResponseStatusCode.get_description(loRa.send_fixed_dict(0, LORA_ADDRESS, LORA_CHANNEL, dictData))



def ToggleBuiltInLed(ledState: bool = None):
    """
    Toggles the Raspberry Pi Pico's built-in LED on or off

    :param bool ledState: A boolean value expressing the power state of the built-in LED
    """

    # Inverse the current power state of the built-in LED
    if ledState is None:
        ledState = not __builtInLed.value()

    __builtInLed.value(ledState)


def __AlarmBuzzerUpdate():
    """
    Toggles the power of the alarm buzzer in a fashion that creates a sound with a particular frequency

    .. note:: The function's while loop update will stop when the alarm buzzer's running state is set to False

    .. seealso:: ToggleAlarmBuzzerState(alarmState: bool)
    """

    global __alarmBuzzerRunning, __alarmBuzzer

    alarmBuzzerSleepTime: int = int(BUZZER_MICROSECONDS / BUZZER_ALARM_HZ / 2)

    while __alarmBuzzerRunning:
        __alarmBuzzer.value(1)
        utime.sleep_us(alarmBuzzerSleepTime)

        __alarmBuzzer.value(0)
        utime.sleep_us(alarmBuzzerSleepTime)


def ToggleAlarmBuzzerState(alarmState: bool = None):
    """
    Toggles the running state of the cansat's alarm buzzer to a given state

    .. note:: This function will create a new thread to operate on when the alarm state is true!

    :param bool alarmState: A boolean value expressing the running state of the cansat's alarm buzzer. When None, the function will simply inverse the current running state
    """

    global __alarmBuzzerRunning

    # Inverse the current running state of the alarm buzzer
    if alarmState is None:
        alarmState = not __alarmBuzzerRunning

    if alarmState and __alarmBuzzerRunning is False:
        __alarmBuzzerRunning = True

        _thread.start_new_thread(__AlarmBuzzerUpdate, ())
    elif alarmState is False and __alarmBuzzerRunning:
        __alarmBuzzerRunning = False
