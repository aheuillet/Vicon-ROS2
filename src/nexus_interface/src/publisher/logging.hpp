///////////////////////////////////////////////////////////////////////////////////////////////
/// This file holds functions related to the logging capabilities. A new log entry can b
/// declared using the log(msg, level) function anywhere in the code.
/// The default log file location, definied in LOG_FILE_PATH is "./client.log".
///////////////////////////////////////////////////////////////////////////////////////////////

#if !defined(LOGGING_H)
#define LOGGING_H

#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <mutex>
#define LOG_FILE_PATH "client.log"

/// Mutex to prevent race condition between two threads trying to simultaneously
/// log something.
static std::mutex log_mtx;

/// Levels of a log entry. Each value of the enum is associated with a string in the
/// log_types array.
enum
{
    INFO = 0,
    WARNING = 1,
    ERROR = 2
};

static std::array<std::string, 3> log_types = {std::string("INFO"), std::string("WARNING"), std::string("ERROR")};

/// Create a new entry in the log file.
/// logMsg: The message to be logged.
/// level: The level of importance of the message (INFO, WARNING or ERROR).
inline void Log(std::string logMsg, int level)
{
    log_mtx.lock();
    std::string filePath = LOG_FILE_PATH;
    std::ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app);
    ofs << " [" + log_types[level] + "] " + logMsg << '\n';
    ofs.close();
    log_mtx.unlock();
}

/// Removes the old log file (if any) and creates a new one.
inline void NewLog()
{
    remove(LOG_FILE_PATH);
    std::string filePath = LOG_FILE_PATH;
    std::ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app);
    ofs << "ROS2VICON 1.0" << '\n';
    ofs.close();
}

#endif // LOGGING_H