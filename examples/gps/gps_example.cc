#include<string>
#include<vector>
#include "../../common/gps.h"
#include <thread>
#include "../../common/map_coordinate.h"
using namespace std;

int main()
{ 
    //const char *path_linux = "/dev/ttyACM0";       // path for linux systems
    const char *path_mac = "/dev/cu.usbmodem141401"; // the path for mac is often different!
    BoBRobotics::GPS::Gps gps(path_mac);  
    
    // if GPS location is invalid, keep trying to get a valid one 
    // if failed 10 times we exit
    int numTrials = 10;
    while(gps.getGPSData().gpsQuality == BoBRobotics::GPS::GPSQuality::INVALID && numTrials > 0) {
        std::this_thread::sleep_for(chrono::milliseconds(500));
        numTrials--;
    } 
     
    if (gps.getGPSData().gpsQuality == BoBRobotics::GPS::GPSQuality::INVALID) {
        std::cout << "invalid GPS location, please wait until surveying finishes or place the antenna to a better spot"<< std::endl;
    } else {
        int i = 0; 
        // print for 100 timestep
        while (i < 100) {  
            try {
                BoBRobotics::GPS::GPSData data = gps.getGPSData();
                BoBRobotics::MapCoordinate::GPSCoordinate coord = data.coordinate;
                BoBRobotics::GPS::TimeStamp time = data.time;
                
                std::cout << "latitude: " << coord.lat.value()<< " longitude: " <<  coord.lon.value() << "[ "
                         <<  time.hour.value() << ":" << time.minute.value() << ":" <<  time.second.value() << ":" << time.millisecond.value() << "] " << std::endl;
                
                i++;
            } catch(...) { }
            // The gps receiver is set to 5HZ
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    
    return 0;

}
