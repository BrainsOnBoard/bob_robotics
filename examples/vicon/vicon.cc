// BoB robotics includes
#include "common/logging.h"
#include "vicon/capture_control.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <chrono>
#include <cstdlib>
#include <thread>

using namespace BoBRobotics::Vicon;
using namespace std::literals;
using namespace units::angle;

int main(int argc, char **argv)
{
    UDPClient<> vicon(51001);
    CaptureControl viconCaptureControl("192.168.1.100", 3003, "c:\\users\\ad374\\Desktop");

    /*
     * If command-line argument is provided, search for an object with specified
     * name, otherwise return first discovered object.
     */
    ObjectData objectData = (argc > 1) ? vicon.getObjectData(argv[1]) : vicon.getObjectData();
    const std::string objectName = objectData.getName();
    LOGI << "Found object: " << objectName;

    viconCaptureControl.startRecording("test1");

    for (int i = 0; i < 10000; i++) {
        auto objectData = vicon.getObjectData(objectName);
        const auto position = objectData.getPosition<>();
        const auto attitude = objectData.getAttitude<degree_t>();
        LOGI << position[0] << ", " << position[1] << ", " << position[2] << ", "
                  << attitude[0] << ", " << attitude[1] << ", " << attitude[2];
    }
    viconCaptureControl.stopRecording("test1");

    return EXIT_SUCCESS;
}
