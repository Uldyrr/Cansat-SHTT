import os


# Values
# // Constants
LOGGER_LOGMAINNAME = "DataLog"
LOGGER_LOGIMUNAME = "IMULog"
LOGGER_LOGFOLDER = "Logs"

LOGTYPE_MAINDATA = "Main"
LOGTYPE_IMUDATA = "IMU"

# // Paths
_workingDirectoryPath = os.getcwd()
_logFolderPath = f"{_workingDirectoryPath}{LOGGER_LOGFOLDER}/"


class CansatLogger:
    _logMainName = ""
    _logIMUName = ""

    def __init__(self):
        self._logMainName = f"{_logFolderPath}{LOGGER_LOGMAINNAME}{len(os.listdir(_logFolderPath)) + 1}.csv"
        self._logIMUName = f"{LOGGER_LOGIMUNAME}.csv"

        # Reset IMU data every restart
        with open(self._logIMUName, "x") as logIMUFile:
            logIMUFile.write("")

    def LogData(self, logType: str, *data: any):
        logStringData = ""

        for i in range(0, len(data) - 1):
            logStringData += str(data[i]) + ("\n" if i == len(data) - 1 else ";")

        with open(self._logMainName if logType == LOGTYPE_MAINDATA else self._logIMUName, "a") as logFile:
            logFile.write(logStringData)

