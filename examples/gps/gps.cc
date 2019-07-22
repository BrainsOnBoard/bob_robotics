// BoB robotics includes
#include "common/gps.h"
#include "common/logging.h"
#include "common/map_coordinate.h"

// Standard C++ includes
#include <string>
#include <vector>
#include <thread>

int main()
{
    const char *path_linux = "/dev/ttyACM0";       // path for linux systems
   // const char *path_mac = "/dev/cu.usbmodem141401"; // the path for mac is often different!

    BoBRobotics::GPS::Gps gps;
    gps.connect(path_linux);

    // if GPS location is invalid, keep trying to get a valid one
    // if failed x times we exit 
    const int maxTrials = 20;
    int numTrials = maxTrials;
    while (numTrials > 0) {
        try {
            if (gps.getGPSData().gpsQuality != BoBRobotics::GPS::GPSQuality::INVALID) {
                std::cout << " we have a valid measurement" << std::endl;
                break;
            }
        } catch(...) {
            std::cout << " measuring failed, trying again in 1 second " << "[" << maxTrials - numTrials << "/" << maxTrials << "]" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            numTrials--;
        }    
    }  
    if (numTrials == 0) {
        std::cout << " There is no valid gps measurement, please try waiting for the survey in to finish and restart the program " << std::endl;
        exit(1);
    }

    int i = 0;
    // print for 100 timestep
    while (i < 100) {
        try {
            BoBRobotics::GPS::GPSData data = gps.getGPSData();
            BoBRobotics::MapCoordinate::GPSCoordinate coord = data.coordinate;
            BoBRobotics::GPS::TimeStamp time = data.time;

            std::cout << "latitude: " << coord.lat.value()<< " longitude: " <<  coord.lon.value() << "[ "
                        <<  time.hour << ":" << time.minute << ":" <<  time.second << ":" << time.millisecond << "] " << std::endl;

            i++;
        } catch(BoBRobotics::GPS::GPSError &e) { LOG_WARNING << e.what(); }
        // The gps receiver is set to 5HZ
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }



    return 0;

}
