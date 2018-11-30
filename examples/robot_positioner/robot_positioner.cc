// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/norbot.h"
#include "robots/robot_positioner.h"
#include "vicon/udp.h"

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
bob_main(int, char **)
{
    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    BoBRobotics::Robots::Norbot bot;

    // setup parameters
    constexpr meter_t stoppingDistance = 10_cm;     // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t allowedHeadingError = 5_deg; // the amount of error allowed in the final heading
    constexpr double k1 = 0.25;                      // curveness of the path to the goal
    constexpr double k2 = 200;                      // speed of turning on the curves
    constexpr double alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases

    BoBRobotics::Robots::RobotPositioner robp(
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

    // drive robot with joystick
    HID::Joystick joystick;
    bot.addJoystick(joystick);

    bool runPositioner = false;
    while (!joystick.isPressed(HID::JButton::B)) {
        const bool joystickUpdate = joystick.update();
        if (joystick.isPressed(HID::JButton::Y)) {
            runPositioner = !runPositioner;
            if (runPositioner) {
                std::cout << "Starting positioner" << std::endl;
            } else {
                bot.stopMoving();
                std::cout << "Stopping positioner" << std::endl;
            }
        }

        if (runPositioner) {
            const auto objectData = vicon.getObjectData(0);
            if (objectData.getElapsedTime() > 10s) {
                bot.stopMoving();
                runPositioner = false;
                std::cerr << "Error: Could not get position from Vicon system\n"
                          << "Stopping trial" << std::endl;
            } else {
                robp.updateMotors(bot, objectData.pose());
            }
        } else if (!joystickUpdate) {
            std::this_thread::sleep_for(5ms);
        }
    }

    return EXIT_SUCCESS;
}
