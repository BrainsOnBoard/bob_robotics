// BoB robotics includes
#include "common/gps_reader.h"
#include "common/map_coordinate.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <string>
#include <thread>
#include <vector>

// Standard C includes
#include <cstring>

using namespace BoBRobotics;
using namespace BoBRobotics::GPS;
using namespace std::literals;

int
bobMain(int argc, char **argv)
{
    const char *path_linux = "/dev/ttyACM0"; // path for linux systems
                                             // const char *path_mac = "/dev/cu.usbmodem141401"; // the path for mac is often different!

    GPSReader gps{ path_linux };

    // if GPS location is invalid, keep trying to get a valid one
    // if failed x times we exit
    const int maxTrials = 20;
    int numTrials = maxTrials;
    GPSData data;
    while (numTrials > 0) {
        try {
            gps.read(data);
            if (data.gpsQuality != GPSQuality::INVALID) {
                std::cout << " we have a valid measurement" << std::endl;
                break;
            }
        } catch (...) {
            std::cout << " measuring failed, trying again in 1 second "
                      << "[" << maxTrials - numTrials << "/" << maxTrials << "]" << std::endl;
            std::this_thread::sleep_for(1s);
            numTrials--;
        }
    }
    if (numTrials == 0) {
        std::cout << " There is no valid gps measurement, please try waiting for the survey in to finish and restart the program " << std::endl;
        return EXIT_FAILURE;
    }

    // Let the user set to nonblocking mode for testing purposes
    if (argc > 1 && strcmp(argv[1], "nonblock") == 0) {
        gps.setBlocking(false);
    }

    // print for 100 timestep
    for (int i = 0; i < 100; i++) {
        try {
            // NB: This will only return false in nonblocking mode
            if (!gps.read(data)) {
                // Indicate that polling has taken place, even though we don't yet have a reading
                std::cout << "(data not yet available)" << std::endl;
                std::this_thread::sleep_for(100ms);
            }

            const auto &coord = data.coordinate;
            const auto &time = data.time;

            std::cout << "latitude: " << coord.lat.value() << " longitude: " << coord.lon.value() << "[ "
                      << time.hour << ":" << time.minute << ":" << time.second << ":" << time.millisecond << "] " << std::endl;
        } catch (BoBRobotics::GPS::GPSError &e) {
            LOG_WARNING << e.what();
        }
    }

    return 0;
}
