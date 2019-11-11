#include "common/map_coordinate.h"
#include <iostream>
#include <string>
#include <cstring>
#include "third_party/UTM.h"
#include <iomanip>

using units::length::meter_t;
using units::angle::degree_t;

using BoBRobotics::MapCoordinate::UTMCoordinate;
using BoBRobotics::MapCoordinate::LatLon;
using BoBRobotics::MapCoordinate::WGS84;
using BoBRobotics::MapCoordinate::Ellipsoid;
using BoBRobotics::MapCoordinate::Cartesian;
using BoBRobotics::MapCoordinate::latLonToCartesian;
using BoBRobotics::MapCoordinate::cartesianToLatLon;
using BoBRobotics::MapCoordinate::shiftLatLon;
using BoBRobotics::MapCoordinate::shiftUTM;

using UTM::LLtoUTM;
using UTM::UTMtoLL;
using CARCoordiante = Cartesian<WGS84>;
using GPSCoordinate = LatLon<WGS84>;

int main()
{
    // test for GPS Coordinate by moving it 10 meters in x direction
    double SHIFT = 30;

    GPSCoordinate home;
    home.lat = degree_t(50.87002350275288);
    home.lon = degree_t(0.0018240860639551215);
    home.height = meter_t(0);
    
    std::cout.precision(17);
    std::cout << "Original Lat: " << std::setw(10)<< home.lat << "\n";
    std::cout << "Original Long: " << std::setw(10) << home.lon << "\n";

    CARCoordiante moveVector;
    moveVector.x = meter_t(SHIFT);
    moveVector.y = meter_t(0);
    moveVector.z = meter_t(0);

    GPSCoordinate target =  shiftLatLon(home,moveVector);
    
    std::cout << "Target Lat: " << target.lat << "\n";
    std::cout << "Target Long: " << target.lon << "\n";
    
    //----------------------------------------------------------------------//
    // test for UTM coordiante
    UTMCoordinate InCoord;
    UTMCoordinate OutCoord;
    InCoord.northing = meter_t(7042000);
    InCoord.easting = meter_t(510000);
    InCoord.height = meter_t(0);
   
    InCoord.zone[0] = '3';
    InCoord.zone[1] = '2';
    InCoord.zone[2] = 'Z';
    InCoord.zone[3] = '\0';

    std::cout << "Origin Northing = " << InCoord.northing.value() << "\n";
    std::cout << "Origin Easting = " << InCoord.easting.value() << "\n";
    
    OutCoord = shiftUTM(InCoord,moveVector);

    std::cout << "Target Northing: " << OutCoord.northing << "\n";
    std::cout << "Target Easting: " << OutCoord.easting << "\n";
    

    return 0;
}