#if !defined(UTILS_HPP)
#define UTILS_HPP
#include <string>
#include <iostream>
#include <fstream>

struct Position
{
    double position_point[2];
    double radius;
} typedef Position;

std::string GetParam(std::string config_file, std::string identifier);


#endif // UTILS_HPP
