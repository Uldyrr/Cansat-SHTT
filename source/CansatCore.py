from __future__ import annotations
from machine import ADC, UART
from imu import MPU6050
from bmp280 import BMP280
from micropyGPS import MicropyGPS
from vector3d import Vector3d


# Module values
# // Sensors
__builtInTemperatureSensor: ADC = ADC(4)

# // Constants
CANSAT_UPDATEHZ: float = 1 / 5  # hz
CANSAT_ALTITUDECORRECTION: float = 120.0  # m

PRESSURE_SEALEVEL: float = 1013.25  # hPa


def GetBuiltInTemperature() -> float:
    """
    Gets the current ambient temperature using the raspberry pico pi's built-in temperature sensor

    :return: Returns a float of the current temperature using the raspberry pico pi's built-in temperature sensor
    """

    adcValue: float = __builtInTemperatureSensor.read_u16()
    voltage: float = (3.3 / 65535) * adcValue
    temperature: float = 27 - (voltage - 0.706) / 0.001721

    return round(temperature, 2)


def GetAltitude(airPressurehPa: float, airTemperatureKelvin: float) -> float:
    """
    Gets the current altitude using the Hypsometric equation (Max height above sea level: ~11km)

    :param float airPressurehPa: The air pressure in hectopascals
    :param float airTemperatureKelvin: The air temperature in kelvin
    :return: Returns a float of the current altitude of the cansat based on the given air pressure and temperature
    """

    pressureRatio: float = PRESSURE_SEALEVEL / airPressurehPa  # sea level pressure = 1013.25 hPa
    altitude: float = (((pressureRatio ** (1 / 5.257)) - 1) * airTemperatureKelvin) / 0.0065 - CANSAT_ALTITUDECORRECTION

    return altitude


def GetMPUAccelerationGyroTemp(mpu: MPU6050, bmp: BMP280, mpuData: dict) -> tuple[Vector3d, Vector3d, float]:
    """
    Gets the current acceleration, gyro, and temperature data using MPU6050 and BMP280

    :param MPU6050 mpu: The mpu6050 sensor connected to the cansat
    :param BMP280 bmp: The bmp280 sensor connected to the cansat
    :param dict mpuData: A dictionary that will be updated to the mpu and bmp's data with keys "Acceleration", "Gyroscope", "Temperature"
    :return: Returns the acceleration data, gyroscope data, and air temperature data
    """

    accelerationData: Vector3d = mpu.accel
    gyroData: Vector3d = mpu.gyro
    airTemperatureData: float = bmp.temperature

    # Update the mpuData dict with MPU6050 data
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

    :param bmp: The bmp280 sensor connected to the cansat
    :param airTemperature: The air temperature measured by the cansat
    :return: Returns the air pressure measured in Pascals and the current altitude of the cansat
    """

    airPressureData: float = bmp.pressure  # Pa

    hpaAirPressure: float = airPressureData * 0.01  # hPa
    airTemperatureKelvin: float = airTemperature + 273.15  # Kelvin

    altitudeData: float = GetAltitude(hpaAirPressure, airTemperatureKelvin)

    return airPressureData, altitudeData


def GetGPSLatitudeLongitude(gps: MicropyGPS, gpsSerialBus: UART) -> tuple[list, list]:
    """
    Uses the cansat's gps module to get the current latitude and longitude of the cansat

    :param gps: The MicropyGPS object of the gps module component (Neo-7m) connected to the cansat
    :param gpsSerialBus: The UART Serial Bus Communication object used to read the gps data
    :return: The latitude and longitude data from the gps
    """

    gpsMessage: bytes | None = gpsSerialBus.readline()

    # Update the MicropyGPS object with data from the gps module component
    if gpsMessage:
        for char in gpsMessage:
            gps.update(chr(char))

    return gps.latitude, gps.longitude
