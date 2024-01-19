import os

# Values
# // Constants
LOGGER_LOGMAINNAME: str = "DataLog"
LOGGER_LOGFOLDER: str = "Logs"

# // Paths
_workingDirectoryPath: str = os.getcwd()
_logFolderPath: str = f"{_workingDirectoryPath}{LOGGER_LOGFOLDER}/"


class CansatLogger:
    _logMainName: str = ""

    def __init__(self):
        self._logMainName = f"{_logFolderPath}{LOGGER_LOGMAINNAME}{len(os.listdir(_logFolderPath)) + 1}.csv"

    def LogData(self, *data: tuple[any, ...]):
        logStringData: str = ""

        for i in range(0, len(data)):
            logStringData += str(data[i]) + (";" if i < len(data) - 1 else "\n")

        with open(self._logMainName, "a") as logFile:
            logFile.write(logStringData)



