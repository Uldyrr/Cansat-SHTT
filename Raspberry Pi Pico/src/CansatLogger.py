import os


# Values
# // Constants
LOGGER_LOGNAME = "DataLog"
LOGGER_LOGFOLDER = "Logs"

# // Paths
_workingDirectoryPath = os.getcwd()
_logFolderPath = f"{_workingDirectoryPath}{LOGGER_LOGFOLDER}/"


class CansatLogger:
    _logName = ""
    _logCount = 0

    def __init__(self):
        self._logName = f"{_logFolderPath}/{LOGGER_LOGNAME}{len(os.listdir(_logFolderPath)) + 1}.csv"

    def LogData(self, *data: any):
        self._logCount += 1

        logStringData = ""

        for i in range(0, len(data) - 1):
            logStringData += str(data[i])  + ("\n" if i == len(data) - 1 else ";")

        with open(self._logName, "a") as logFile:
            logFile.write(logStringData)

