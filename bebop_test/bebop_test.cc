// C++ includes
#include <chrono>
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

    drone.addJoystick(joystick, /*maxSpeed=*/0.25);
    Video::Display display(drone.getVideoStream());
    do {
        if (!joystick.update() && !display.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    } while (display.isOpen());
}
