import os

# Values
# // Constants
LOGGER_LOGMAINNAME: str = "DataLog"
LOGGER_LOGFOLDER: str = "Logs"
LOGGER_CSVSEPERATOR: str = ";"

# // Paths
_workingDirectoryPath: str = os.getcwd()
_logFolderPath: str = f"{_workingDirectoryPath}{LOGGER_LOGFOLDER}/"


class CansatLogger:
    _logFileMainName: str = ""

    def __init__(self):
        self._logFileMainName = f"{_logFolderPath}{LOGGER_LOGMAINNAME}{len(os.listdir(_logFolderPath)) + 1}.csv"

    def LogData(self, *data: any):
        logStringData: str = ""

        for i in range(0, len(data)):
            logStringData += str(data[i]) + (LOGGER_CSVSEPERATOR if i < len(data) - 1 else "\n")

        with open(self._logFileMainName, "a") as logFile:
            logFile.write(logStringData)
