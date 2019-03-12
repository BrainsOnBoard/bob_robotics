// BoB robotics includes
#include "vicon/capture_control.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace BoBRobotics::Vicon;
using namespace std::literals;
using namespace units::angle;

int main(int argc, char **argv)
{
    UDPClient<> vicon(51001);
    CaptureControl viconCaptureControl("192.168.1.100", 3003, "c:\\users\\ad374\\Desktop");

    const std::string objectName = (argc > 1) ? argv[1] : "Norbot";

    size_t norbotID = vicon.findObjectID(objectName);
    std::cout << "'" << objectName << "' found with id: " << norbotID << std::endl;

    if (!viconCaptureControl.startRecording("test1")) {
        return EXIT_FAILURE;
    }
    for (int i = 0; i < 1000; i++) {
        auto objectData = vicon.getObjectData(norbotID);
        const auto position = objectData.getPosition<>();
        const auto attitude = objectData.getAttitude<degree_t>();
        std::cout << position[0] << ", " << position[1] << ", " << position[2] << ", "
                  << attitude[0] << ", " << attitude[1] << ", " << attitude[2] << std::endl;
    }
    if (!viconCaptureControl.stopRecording("test1")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
