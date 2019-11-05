#include "common/map_coordinate.h"
#include <iostream>
#include <string>
#include <cstring>
#include "third_party/UTM.h"
using BoBRobotics::MapCoordinate::UTMCoordinate;
using units::length::meter_t; 

UTMCoordinate Coord1;

int main()
{
    Coord1.easting = meter_t(1);
    Coord1.northing = meter_t(2);
    Coord1.height = meter_t(3);
    
    Coord1.zone[0] = '3';
    Coord1.zone[1] = '0';
    Coord1.zone[2] = 'Z';
    
    std::cout << "Easting: " << Coord1.easting << "\n";
    std::cout << "Northing: " << Coord1.northing << "\n";
    std::cout << "Height: " << Coord1.height << "\n";
    std::cout << "Zone: " << Coord1.zone << "\n";
    
    
    return 0;
}