// C++ includes
#include <chrono>
#include <iostream>
#include <thread>

// BoB robotics includes
#include "vicon/capture_control.h"
#include "vicon/object_plotter.h"
#include "vicon/udp.h"

using namespace BoBRobotics::Vicon;
using namespace std::literals;
using namespace units::angle;

auto now()
{
    return std::chrono::high_resolution_clock::now();
}

int
main()
{
    UDPClient<ObjectPlotter<>> vicon(51001);
    CaptureControl viconCaptureControl("192.168.1.100", 3003, "c:\\users\\ad374\\Desktop");
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    if (!viconCaptureControl.startRecording("test1")) {
        return EXIT_FAILURE;
    }

    auto &plotter = vicon.getObject(0);
    do {
        if (!plotter.update()) {
            std::this_thread::sleep_for(25ms);
        }
    } while (plotter.isOpen());

    if (!viconCaptureControl.stopRecording("test1")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
