// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// BoB robotics includes
#include "vicon/capture_control.h"
#include "vicon/udp.h"

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::angle;
using namespace units::length;

int
main()
{
    Vicon::UDPClient<> vicon(51001);
    Vicon::CaptureControl viconCaptureControl("192.168.1.100", 3003, "c:\\users\\ad374\\Desktop");
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }
    std::cout << "Object name: " << vicon.getObjectData(0).getObjectName() << std::endl;

    if (!viconCaptureControl.startRecording("test1")) {
        return EXIT_FAILURE;
    }
    for (int i = 0; i < 10000; i++) {
        const Pose3<millimeter_t, degree_t> data = vicon.getObjectData(0);
        std::cout << data.x() << ", " << data.y() << ", " << data.z() << ", "
                  << data.yaw() << ", " << data.pitch() << ", " << data.roll() << std::endl;
    }
    if (!viconCaptureControl.stopRecording("test1")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
