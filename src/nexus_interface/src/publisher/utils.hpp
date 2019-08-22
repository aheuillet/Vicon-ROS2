///////////////////////////////////////////////////////////////////////////////////////////////
/// This file holds miscellaneous functions, mostly related to the config file management.
///////////////////////////////////////////////////////////////////////////////////////////////

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

/// Struct used to hold segment data to transmit to the Publisher class.
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

/// Struct representing a config file line.
struct ConfigLine
{
    std::string name;
    std::string value;
} typedef ConfigLine;

/// Writes the given list of ConfigLine structs to the app's config file.
/// The path to the config file is defined in CONFIG_FILE_LOCATION.
void WriteConfigLines(std::list<ConfigLine> lines);

/// Parses the config file to get a list of ConfigLine structs.
std::list<ConfigLine> GetConfigLines();

/// Get the string value associated to the given identifier in the app's config file.
std::string GetParam(std::string identifier);

/// Check if an occurence of str2 is present in str1 (case insensitive).
bool ci_find_substr(std::string str1, std::string str2);

#endif // UTILS_HPP
