from os import path
import os


# Values
# // Constants
LOGGER_LOGNAME = "DataLog"
LOGGER_LOGFOLDER = "Logs"

# // Paths
_workingDirectoryPath = os.getcwd()
_logFolderPath = path.join(_workingDirectoryPath, LOGGER_LOGFOLDER)


class CansatLogger:
    _logName = path.join(_logFolderPath, f"{LOGGER_LOGNAME}1.txt")

    def __init__(self):
        self._GetAvailableLogName()

    def _GetAvailableLogName(self) -> None:
        logId = 1

        while path.exists(self._logName):
            logId += 1
            self._logName = path.join(_logFolderPath, f"{LOGGER_LOGNAME}{logId}.txt")

    def LogData(self, data: str):
        with open(self._logName, "w") as logFile:
            logFile.write(f"\n{data}")

