// BoB robotics includes
#include "common/macros.h"
#include "common/map_coordinate.h"
#include "gps/gps_reader.h"

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
#ifdef __linux__
    BOB_ASSERT(argc <= 2);
    const char *devicePath = (argc == 2) ? argv[1] : GPSReader::DefaultLinuxDevicePath;
#else
    /*
     * Need to provide path explicitly on macOS!
     * (e.g.: ./gps /dev/cu.usbmodem141401)
     */
    BOB_ASSERT(argc == 2);

    const char *devicePath = argv[1];
#endif

    GPSReader gps{ devicePath };

    // Let the user set to nonblocking mode for testing purposes
    if (argc > 1 && strcmp(argv[1], "nonblock") == 0) {
        gps.setBlocking(false);
    }

    // print for 100 timestep
    GPSData data;
    for (int i = 0; i < 100; i++) {
        // NB: This will only return false in nonblocking mode
        if (!gps.read(data)) {
            // Indicate that polling has taken place, even though we don't yet have a reading
            std::cout << "(data not yet available)" << std::endl;
            std::this_thread::sleep_for(100ms);
            continue;
        }

        const auto &coord = data.coordinate;
        const auto &time = data.time;

        std::cout << "latitude: " << coord.lat.value() << " longitude: " << coord.lon.value() << "[ "
                  << time.hour << ":" << time.minute << ":" << time.second << ":" << time.millisecond << "] " << std::endl;
    }

    return 0;
}
