#if !defined(UTILS_HPP)
#define UTILS_HPP
#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "logging.hpp"

#define CONFIG_FILE_LOCATION "./settings.cfg"
#define POSITION_NUMBER 3
#define BODY_PARTS_NUMBER 10

struct Position
{
    double translation[3];
    double rotation[4];
    std::string subject_name;
    std::string segment_name;
    std::string translation_type;
    unsigned int frame_number;

    std::string toString() 
    {
        return "X: " + std::to_string(translation[0]) + " Y: " + std::to_string(translation[1]) + " Z: " + std::to_string(translation[2]); 
    }
} typedef Position;

struct RootSegment 
{
    std::string name;
    std::string subject_name; 
} typedef RootSegment;

struct ConfigLine 
{
    std::string name;
    std::string value; 
} typedef ConfigLine;

void WriteConfigLines(std::list<ConfigLine> lines);

std::list<ConfigLine> GetConfigLines();

std::string GetParam(std::string identifier);

bool ci_find_substr(std::string str1, std::string str2);


#endif // UTILS_HPP
