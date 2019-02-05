// BoB robotics includes
#include "robots/tank_pid.h"
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "vicon/udp.h"

// This program can be run locally on the robot or remotely
#ifdef NO_I2C_ROBOT
#include "net/client.h"
#include "robots/tank_netsink.h"
#else
#include "robots/norbot.h"
#endif

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace units::length;
using namespace units::literals;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::velocity;
using namespace std::literals;

int
bob_main(int argc, char **argv)
{
    // Parameters
    constexpr meter_t stoppingDistance = 10_cm; // if the robot's distance from goal < stopping dist, robot stops
    constexpr float kp = 0.1f;
    constexpr float ki = 0.1f;
    constexpr float kd = 0.1f;
    constexpr float averageSpeed = 0.5f;

#ifdef NO_I2C_ROBOT
    std::string robotIP;
    if (argc == 2) {
        // Get robot IP from command-line argument
        robotIP = argv[1];
    } else {
        // Get robot IP from terminal
        std::cout << "Robot IP [10.0.0.3]: ";
        std::getline(std::cin, robotIP);
        if (robotIP.empty()) {
            robotIP = "10.0.0.3";
        }
    }

    // Make connection to robot on default port
    Net::Client client(robotIP);
    Robots::TankNetSink robot(client);
#else
    // Silence warning about unused vars
    (void) argc;
    (void) argv;

    // Connect to motors over I2C
    Robots::Norbot robot;
#endif

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    // Drive robot with joystick
    HID::Joystick joystick;
    robot.addJoystick(joystick);

    // Print distance timer
    Stopwatch printTimer;

    {
        // Catch exceptions on background threads
        BackgroundExceptionCatcher catcher;
        catcher.trapSignals(); // Trap signals e.g. ctrl+c

#ifdef NO_I2C_ROBOT
        // Read on background thread
        client.runInBackground();
#endif

        Robots::TankPID pid(kp, ki, kd, averageSpeed);

        // set goal pose
        const Vector2<millimeter_t> goal{ 0_mm, 0_mm };
        std::cout << "Goal: (" << goal.x() << ", " << goal.y() << ")" << std::endl;
        std::cout << "Press Y to start homing" << std::endl;

        bool runPositioner = false;
        while (!joystick.isPressed(HID::JButton::B)) {
            // Check for background exceptions
            catcher.check();

            // Poll for joystick events
            const bool joystickUpdate = joystick.update();
            if (joystick.isPressed(HID::JButton::Y)) {
                runPositioner = !runPositioner;
                if (runPositioner) {
                    std::cout << "Starting positioner" << std::endl;
                    printTimer.start();
                    pid.start();
                } else {
                    robot.stopMoving();
                    std::cout << "Stopping positioner" << std::endl;
                }
            }

            // Get motor commands from positioner, if it's running
            if (runPositioner) {
                const auto objectData = vicon.getObjectData(0);
                if (objectData.timeSinceReceived() > 10s) {
                    robot.stopMoving();
                    runPositioner = false;
                    std::cerr << "Error: Could not get position from Vicon system\n"
                              << "Stopping trial" << std::endl;
                    continue;
                }

                const auto position = objectData.getPosition();
                const auto attitude = objectData.getAttitude();
                const auto distance = goal.distance2D(position);
                if (distance <= stoppingDistance) {
                    std::cout << "Reached goal" << std::endl;
                    std::cout << "Final position: " << position.x() << ", " << position.y() << std::endl;
                    std::cout << "Goal: " << goal.x() << ", " << goal.y() << std::endl;
                    std::cout << "Distance to goal: "
                              << goal.distance2D(position)
                              << " (" << goal.yaw() - attitude[0] << ")"
                              << std::endl;
                    robot.stopMoving();
                    runPositioner = false;
                } else {
                    pid.drive(robot, objectData.getPose(), goal);

                    // Print status
                    if (printTimer.elapsed() > 500ms) {
                        printTimer.start();
                        std::cout << "Distance to goal: "
                                  << goal.distance2D(position)
                                  << " (" << goal.yaw() - attitude[0] << ")"
                                  << std::endl;
                    }
                }
            } else if (!joystickUpdate) {
                std::this_thread::sleep_for(5ms);
            }
        }
    }

    return EXIT_SUCCESS;
}
