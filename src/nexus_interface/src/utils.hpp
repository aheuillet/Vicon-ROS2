#if !defined(UTILS_HPP)
#define UTILS_HPP
#include <string>
#include <iostream>
#include <fstream>

struct Position
{
    double position_point[2];
    double radius;

    std::string toString() 
    {
        return "First point: " + std::to_string(position_point[0]) + " Second point: " + std::to_string(position_point[1]) + " Radius: " + std::to_string(radius); 
    }
} typedef Position;

std::string GetParam(std::string config_file, std::string identifier);


#endif // UTILS_HPP
