#if !defined(UTILS_HPP)
#define UTILS_HPP
#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include "logging.hpp"

#define CONFIG_FILE_LOCATION "./settings.cfg"
#define POSITION_NUMBER 3

struct Position
{
    double translation[3];

    std::string toString() 
    {
        return "X: " + std::to_string(translation[0]) + " Y: " + std::to_string(translation[1]) + " Z: " + std::to_string(translation[2]); 
    }
} typedef Position;

struct ConfigLine 
{
    std::string name;
    std::string value; 
} typedef ConfigLine;

void WriteConfigLines(std::list<ConfigLine> lines);

std::list<ConfigLine> GetConfigLines();

std::string GetParam(std::string identifier);


#endif // UTILS_HPP
