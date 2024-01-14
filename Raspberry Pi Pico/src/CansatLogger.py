import os


# Values
# // Constants
LOGGER_LOGMAINNAME = "DataLog"
LOGGER_LOGIMUNAME = "IMULog"
LOGGER_LOGFOLDER = "Logs"

# // Paths
_workingDirectoryPath = os.getcwd()
_logFolderPath = f"{_workingDirectoryPath}{LOGGER_LOGFOLDER}/"


class CansatLogger:
    _logMainName = ""
    _logIMUName = ""

    def __init__(self):
        logDataID = len(os.listdir(_logFolderPath)) + 1

        self._logMainName = f"{_logFolderPath}{LOGGER_LOGMAINNAME}{logDataID}.csv"
        self._logIMUName = f"{_logFolderPath}{LOGGER_LOGIMUNAME}{logDataID}.csv"

    def LogData(self, *data: any):
        logStringData = ""

        for i in range(0, len(data) - 1):
            logStringData += str(data[i]) + ("\n" if i == len(data) - 1 else ";")

        with open(self._logName, "a") as logFile:
            logFile.write(logStringData)

