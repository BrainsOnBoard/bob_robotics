#include<string>
#include<vector>
#include "../../common/gps.h"

using namespace std;

int main()
{ 
   
    const char *path = "/dev/ttyACM0";
    Gps gps(path);
    units::angle::degree_t lat;
    units::angle::degree_t lon;
    units::length::meter_t alt;  
    while (1) {
        gps.getPosition(lat,lon, alt);
        std::cout << "latitude: " << lat.value() << " longitude: " <<  lon.value() << " altitude: " << alt << std::endl;
        Gps::GPSQuality qual = gps.getGpsQuality();
    }

    
    return 0;

}
