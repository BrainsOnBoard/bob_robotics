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

using UTM::LLtoUTM;
using UTM::UTMtoLL;

using CARCoordiante = Cartesian<WGS84>;
using GPSCoordinate = LatLon<WGS84>;

template <typename T>
LatLon<T> updateGoalLocation(LatLon<T> origin, Cartesian<T> moveVector)
{
    GPSCoordinate target;
    CARCoordiante temp = latLonToCartesian(origin) + moveVector;
    target = cartesianToLatLon(temp);
    return target;
};

//template <typename T>
//UTMCoordinate updateGoalLocation(UTMCoordinate origin, Cartesian<T> moveVector)
//{

    // Get a UTMCoordinate origin

    // Transform it to LL

    // Transform to CAR

    // Do shift

    // Retransform To LL

    // Retransform to UTM

    // return


    // LLtoUTM (Lat, Long, &Norhing, &Easting, Zone)
    // UTMtoLL (Northing, Easting, Zone, type &Lat , type&Long)
    // GPSCoordinate target =  UTMtoLL(origin);
    // CARCoordiante temp = latLonToCartesian(origin) + moveVector;
    // UTMCoordinate target = LLtoUTM(cartesianToLatLon(temp).lat,);
    // return target;
//};


int main()
{
        
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
    
    // Create myself a cartesian coordinate

    // UTM -> Cart
    // Cart = Cart + x
    // Cart -> UTM

   UTMCoordinate test;
   test.northing = meter_t(7042000);
   test.easting = meter_t(510000);
   
    test.zone[0] = '3';
    test.zone[1] = '2';
    test.zone[2] = 'Z';
    test.zone[3] = '\0';

    GPSCoordinate result;
    result.lat = degree_t(0);
    result.lon = degree_t(1);

    double lat = 63.506144;
    double lon = 9.20091;

    

    UTMtoLL(test.northing.value(),test.easting.value(),
            test.zone,lat,lon);

    std::cout << "Lat = " << lat << "\n";
    std::cout << "Lon = " << lon << "\n";
    //std::cout << "New\n";
    return 0;
}