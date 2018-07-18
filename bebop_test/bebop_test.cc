// C++ includes
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

// GeNN robotics includes
#include "hid/joystick.h"
#include "robots/bebop.h"
#include "video/display.h"

using namespace BoBRobotics;
using namespace std::literals;

template <class T>
void print(T limits)
{
    std::cout << " (limits: " << std::get<0>(limits) << ", "
              << std::get<1>(limits) << ")" << std::endl;
}

void printSpeedLimits(Robots::Bebop &drone)
{
    // max tilt
    auto maxTilt = drone.getMaximumTilt();
    auto tiltLimits = drone.getTiltLimits();
    std::cout << "Max tilt: " << maxTilt;
    print(tiltLimits);
    
    // max yaw speed
    auto maxYawSpeed = drone.getMaximumYawSpeed();
    auto yawLimits = drone.getYawSpeedLimits();
    std::cout << "Max yaw speed: " << maxYawSpeed;
    print(yawLimits);

    // max vertical speed
    auto maxVertSpeed = drone.getMaximumVerticalSpeed();
    auto vertSpeedLimits = drone.getVerticalSpeedLimits();
    std::cout << "Max vertical speed: " << maxVertSpeed;
    print(vertSpeedLimits);
}

int main()
{
    HID::Joystick joystick(/*deadZone=*/0.25);

    Robots::Bebop drone;
    drone.connect();

    try {
        printSpeedLimits(drone);
        drone.addJoystick(joystick, /*maxSpeed=*/1.0);

        Video::Display display(drone.getVideoStream());
        do {
            if (!joystick.update() && !display.update()) {
                std::this_thread::sleep_for(25ms);
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
