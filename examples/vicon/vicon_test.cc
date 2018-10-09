// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// BoB robotics includes
#include "vicon/capture_control.h"
#include "vicon/udp.h"

using namespace BoBRobotics::Vicon;
using namespace std::literals;
using namespace units::angle;

int
main()
{
    UDPClient<> vicon(51001);
    CaptureControl viconCaptureControl("192.168.1.100", 3003, "c:\\users\\ad374\\Desktop");
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    if (!viconCaptureControl.startRecording("test1")) {
        return EXIT_FAILURE;
    }
    auto &object = vicon.getObject(0);
    for (int i = 0; i < 10000; i++) {
        const auto position = object.getPosition<>();
        const auto attitude = object.getAttitude<degree_t>();
        std::cout << position[0] << ", " << position[1] << ", " << position[2] << ", "
                  << attitude[0] << ", " << attitude[1] << ", " << attitude[2] << std::endl;
    }
    if (!viconCaptureControl.stopRecording("test1")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
