// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
#include "libbebop/bebop.h"
#include "video/display.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;

template<class T>
void
print(T limits)
{
    std::cout << " (limits: " << limits.first << ", "
              << limits.second << ")" << std::endl;
}

void
printSpeedLimits(Bebop &drone)
{
    // max tilt
    auto maxTilt = drone.getMaximumTilt();
    auto tiltLimits = drone.getTiltLimits();
    std::cout << "Max tilt: " << maxTilt;
    print(tiltLimits);

    // max yaw speed
    auto maxTurnSpeed = drone.getMaximumTurnSpeed();
    auto yawLimits = drone.getTurnSpeedLimits();
    std::cout << "Max yaw speed: " << maxTurnSpeed;
    print(yawLimits);

    // max vertical speed
    auto maxVertSpeed = drone.getMaximumVerticalSpeed();
    auto vertSpeedLimits = drone.getVerticalSpeedLimits();
    std::cout << "Max vertical speed: " << maxVertSpeed;
    print(vertSpeedLimits);
}

int
bob_main(int, char **)
{
    /*
     * Connects to the drone.
     *
     * NB: Any or all of these parameters can be omitted to use the defaults,
     *     which is probably almost always what you want. Side note: don't set
     *     these values to their maximum if you want to be able to control the
     *     drone.
     */
    Bebop drone(/*maxTurnSpeed=*/Bebop::DefaultMaximumTurnSpeed,
                /*maxVerticalSpeed=*/Bebop::DefaultMaximumVerticalSpeed,
                /*maxTilt=*/Bebop::DefaultMaximumTilt);

    std::cout << "Battery is at: " << drone.getBatteryLevel() * 100.f << "%" << std::endl;

    // print maximum speed parameters
    printSpeedLimits(drone);

    // control drone with joystick
    HID::Joystick joystick(/*deadZone=*/0.25);
    drone.addJoystick(joystick);

    // display the drone's video stream on screen
    Video::Display display(drone.getVideoStream(), true);
    do {
        bool joyUpdated = joystick.update();
        bool dispUpdated = display.update();
        if (!joyUpdated && !dispUpdated) {
            std::this_thread::sleep_for(25ms);
        }
    } while (display.isOpen());

    return EXIT_SUCCESS;
}
