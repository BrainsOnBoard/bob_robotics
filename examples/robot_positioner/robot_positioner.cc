// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/robot_positioner.h"
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
    constexpr meter_t stoppingDistance = 10_cm;     // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t allowedHeadingError = 5_deg; // the amount of error allowed in the final heading
    constexpr double k1 = 0.025;                    // curveness of the path to the goal
    constexpr double k2 = 400;                      // speed of turning on the curves
    constexpr double alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases

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
    Robots::TankNetSink bot(client);
#else
    // Silence warning about unused vars
    (void) argc;
    (void) argv;

    // Connect to motors over I2C
    Robots::Norbot bot;
#endif

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    // drive robot with joystick
    HID::Joystick joystick;
    bot.addJoystick(joystick);

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

        Robots::RobotPositioner robp(
                stoppingDistance,
                allowedHeadingError,
                k1,
                k2,
                alpha,
                beta,
                bot.getMaximumSpeed(),
                bot.getMaximumTurnSpeed());

        // set goal pose
        const Pose2<millimeter_t, degree_t> goal{ 0_mm, 0_mm, 15_deg };
        std::cout << "Goal: (" << goal.x() << ", " << goal.y() << ") at " << goal.yaw() << std::endl;
        robp.setGoalPose(goal);
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
                } else {
                    bot.stopMoving();
                    std::cout << "Stopping positioner" << std::endl;
                }
            }

            // Get motor commands from positioner, if it's running
            if (runPositioner) {
                const auto objectData = vicon.getObjectData(0);
                const auto position = objectData.getPosition();
                const auto attitude = objectData.getAttitude();
                if (objectData.timeSinceReceived() > 10s) {
                    bot.stopMoving();
                    runPositioner = false;
                    std::cerr << "Error: Could not get position from Vicon system\n"
                              << "Stopping trial" << std::endl;
                } else if (robp.reachedGoal()) {
                    std::cout << "Reached goal" << std::endl;
                    std::cout << "Final position: " << position.x() << ", " << position.y() << std::endl;
                    std::cout << "Goal: " << goal.x() << ", " << goal.y() << std::endl;
                    std::cout << "Distance to goal: "
                              << goal.distance2D(position)
                              << " (" << goal.yaw() - attitude[0] << ")"
                              << std::endl;
                    runPositioner = false;
                } else {
                    robp.updateMotors(bot, { position[0], position[1], attitude[0] });

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
