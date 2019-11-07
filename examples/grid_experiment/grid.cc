#include "common/map_coordinate.h"
#include <iostream>
#include <string>
#include <cstring>
#include "third_party/UTM.h"
#include <iomanip>
using BoBRobotics::MapCoordinate::UTMCoordinate;
using units::length::meter_t;
using units::angle::degree_t;
using BoBRobotics::MapCoordinate::LatLon;
using BoBRobotics::MapCoordinate::WGS84;
using BoBRobotics::MapCoordinate::Ellipsoid;
using GPSCoordinate = LatLon<WGS84>;
using BoBRobotics::MapCoordinate::latLonToCartesian;
using BoBRobotics::MapCoordinate::cartesianToLatLon;
using BoBRobotics::MapCoordinate::Cartesian;
using CARCoordiante = Cartesian<WGS84>;


UTMCoordinate UTMCoord; 
GPSCoordinate H;
GPSCoordinate updateGoalLocation(GPSCoordinate origin, CARCoordiante moveVector)
{
    GPSCoordinate target;
    CARCoordiante temp = latLonToCartesian(origin) + moveVector;
    target = cartesianToLatLon(temp);
    return target;
};

int main()
{
        
    // // Create myself a UTM based coordinate
    // UTMCoord.easting = meter_t(1);
    // UTMCoord.northing = meter_t(2);
    // UTMCoord.height = meter_t(3);
    
    // UTMCoord.zone[0] = '3';
    // UTMCoord.zone[1] = '0';
    // UTMCoord.zone[2] = 'Z';
    
    // Create myself a LatLon based coordinate
    
    GPSCoordinate home;
    home.lat = degree_t(50.87002350275288);
    home.lon = degree_t(0.0018240860639551215);
    home.height = meter_t(0);
    std::cout.precision(17);
    std::cout << "Original Lat: " << std::setw(10)<<home.lat*RAD_TO_DEG << "\n";
    std::cout << "Original Long: " << std::setw(10) << home.lon*RAD_TO_DEG << "\n";

    CARCoordiante moveVector;
    moveVector.x = meter_t(10);
    moveVector.y = meter_t(0);
    moveVector.z = meter_t(0);

    GPSCoordinate target =  updateGoalLocation(home,moveVector);
    
    std::cout << "New Lat: " << target.lat*RAD_TO_DEG << "\n";
    std::cout << "New Long: " << target.lon*RAD_TO_DEG << "\n";
    
    std::cout << "I ran!" << "\n";

    // Create myself a cartesian coordinate

    // UTM -> Cart
    // Cart = Cart + x
    // Cart -> UTM

   
    return 0;
}