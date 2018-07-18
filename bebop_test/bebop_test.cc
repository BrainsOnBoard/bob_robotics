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
using namespace BoBRobotics::Robots;
using namespace std::literals;

template <class T>
void print(T limits)
{
    std::cout << " (limits: " << std::get<0>(limits) << ", "
              << std::get<1>(limits) << ")" << std::endl;
}

void printSpeedLimits(Bebop &drone)
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
    // joystick for controlling drone
    HID::Joystick joystick(/*deadZone=*/0.25);

    /*
     * Initialises the drone object and searches for the drone on the network,
     * but does not connect to the drone yet.
     * 
     * NB: Any or all of these parameters can be omitted to use the defaults,
     *     which is probably almost always what you want. Side note: don't set
     *     these values to their maximum if you want to be able to control the
     *     drone.
     */
    Bebop drone(/*maxYawSpeed=*/Bebop::DefaultMaximumYawSpeed,
                /*maxVerticalSpeed=*/Bebop::DefaultMaximumVerticalSpeed,
                /*maxTilt=*/Bebop::DefaultMaximumTilt);

    // make connection to drone
    drone.connect();

    try {
        // print maximum speed parameters
        printSpeedLimits(drone);

        // control drone with joystick
        drone.addJoystick(joystick);

        // display the drone's video stream on screen
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
