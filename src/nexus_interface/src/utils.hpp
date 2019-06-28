#if !defined(UTILS_HPP)
#define UTILS_HPP
#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include "logging.hpp"

#define CONFIG_FILE_LOCATION "./settings.cfg"

struct Position
{
    double position_point[2];
    double radius;

    std::string toString() 
    {
        return "First point: " + std::to_string(position_point[0]) + " Second point: " + std::to_string(position_point[1]) + " Radius: " + std::to_string(radius); 
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
