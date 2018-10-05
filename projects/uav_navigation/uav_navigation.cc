// BoB robotics includes
#include "hid/joystick.h"
#include "libbebop/bebop.h"
#include "video/display.h"

// Standard C++ includes
#include <exception>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

int
main()
{
    HID::Joystick joystick;

    std::cout << "Connecting..." << std::endl;
    Robots::Bebop drone;
    std::cout << "Connected to drone" << std::endl;

    try {
        bool stopflag = false;
        drone.setFlightEventHandler([&stopflag](bool takeoff) {
            if (!takeoff) {
                stopflag = true;
            }
        });

        auto &camera = drone.getVideoStream();
        Video::Display display(camera);

        drone.addJoystick(joystick);
        do {
            const bool joystickUpdate = joystick.update();
            const bool displayUpdate = display.update();
            if (!joystickUpdate && !displayUpdate) {
                std::this_thread::sleep_for(25ms);
            }
        } while (!stopflag && display.isOpen());
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}