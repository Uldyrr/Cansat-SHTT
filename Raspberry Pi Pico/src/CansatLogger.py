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
        self._logName = f"{_logFolderPath}/{LOGGER_LOGNAME}{len(os.listdir(_logFolderPath)) + 1}.txt"

    def LogData(self, data: str):
        self._logCount += 1

        data = "".join(data.splitlines())#fjeijfei

        with open(self._logName, "a") as logFile:
            logFile.write(f"ID: {self._logCount} | Data: {data}\n")

