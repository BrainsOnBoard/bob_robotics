// C++ includes
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

// GeNN robotics includes
#include "hid/joystick.h"
#include "robots/bebop.h"
#include "video/display.h"

using namespace GeNNRobotics;

int main()
{
    HID::Joystick joystick(/*deadZone=*/0.25);

    Robots::Bebop drone;
    drone.connect();

    try {
        drone.addJoystick(joystick, /*maxSpeed=*/0.25);
        Video::Display display(drone.getVideoStream());
        do {
            if (!joystick.update() && !display.update()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
        } while (display.isOpen());
    } catch (std::exception &e) {
        /*
         * We catch all exceptions, because otherwise drone's destructor won't
         * be called and the drone might be left hovering (or flying towards a
         * wall).
         */
        std::cerr << "ERROR: Uncaught exception: " << e.what() << std::endl;
    }
}
