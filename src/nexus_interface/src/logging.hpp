#if !defined(LOGGING_H)
#define LOGGING_H

#include <iostream>
#include <fstream>
#include <string>
#include <array>

enum {
    INFO=0,
    WARNING=1,
    ERROR=2
};

static std::array<std::string, 3> log_types = {std::string("INFO"), std::string("WARNING"), std::string("ERROR")};

inline void Log(std::string logMsg, int level)
{
    std::string filePath = "client.log";
    std::ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app);
    ofs << " [" + log_types[level] + "] " + logMsg << std::endl;
    ofs.close();
}

inline void NewLog()
{
    remove("client.log");
    std::string filePath = "client.log";
    std::ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app);
    ofs << "Starting Client" << std::endl;
    ofs.close();
}

#endif // LOGGING_H