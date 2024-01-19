import os


# Values
# // Constants
LOGGER_LOGMAINNAME: str = "DataLog"
LOGGER_LOGIMUNAME: str = "IMULog"
LOGGER_LOGFOLDER: str = "Logs"

LOGTYPE_MAINDATA: str = "Main"
LOGTYPE_IMUDATA: str = "IMU"

# // Paths
_workingDirectoryPath: str = os.getcwd()
_logFolderPath: str = f"{_workingDirectoryPath}{LOGGER_LOGFOLDER}/"


class CansatLogger:
    _logMainName: str = ""
    _logIMUName: str = ""

    def __init__(self):
        self._logMainName = f"{_logFolderPath}{LOGGER_LOGMAINNAME}{len(os.listdir(_logFolderPath)) + 1}.csv"
        self._logIMUName = f"{LOGGER_LOGIMUNAME}.csv"

        # Reset IMU data every restart
        with open(self._logIMUName, "w") as logIMUFile:
            logIMUFile.write("")

    def LogData(self, logType: str, *data: tuple[any, ...]):
        logStringData: str = ""

        for i in range(0, len(data)):
            logStringData += str(data[i]) + (";" if i < len(data) - 1 else "\n")

        with open(self._logMainName if logType == LOGTYPE_MAINDATA else self._logIMUName, "a") as logFile:
            logFile.write(logStringData)


