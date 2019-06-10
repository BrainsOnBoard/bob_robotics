// BoB robotics includes
#include "common/logging.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/bebop/bebop.h"
#include "robots/control/uav_positioner.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;
using namespace units::length;
using namespace units::angle;

int
bob_main(int, char **)
{
    // drone object
    Bebop drone;
    std::atomic<bool> stopFlag{ false };
    drone.setFlightEventHandler([&stopFlag](bool takeoff) {
        if (!takeoff) {
            stopFlag = true;
        }
    });

    // Vicon system
    Vicon::UDPClient<Vicon::ObjectDataVelocity> vicon{ 51001 };

    // for moving drone to a location
    UAVPositioner positioner{ drone, vicon };

    // control drone with joystick
    HID::Joystick joystick;
    drone.addJoystick(joystick);
    joystick.runInBackground();

    std::string str;
    Stopwatch stopwatch;
    while (!stopFlag) {
        {
            std::cout << "Enter x: ";
            std::getline(std::cin, str);
            if (str.empty()) {
                return 0;
            }
            meter_t x(stod(str));

            std::cout << "Enter y: ";
            std::getline(std::cin, str);
            if (str.empty()) {
                return 0;
            }
            meter_t y(stod(str));

            std::cout << "Enter z: ";
            std::getline(std::cin, str);
            if (str.empty()) {
                return 0;
            }
            meter_t z(stod(str));

            std::cout << "Enter yaw change: ";
            std::getline(std::cin, str);
            if (str.empty()) {
                return 0;
            }
            degree_t yawChange(stod(str));

            LOGI << "Starting move...";
            stopwatch.start();
            while (stopwatch.elapsed() < 30s && !stopFlag) {
                positioner.update();
                std::this_thread::sleep_for(25ms);
            }
        }
        if (stopFlag) {
            break;
        }
    }

    return EXIT_SUCCESS;
}
