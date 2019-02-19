#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "common/background_exception_catcher.h"
#include "common/circstat.h"
#include "common/main.h"
#include "common/plot_agent.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/tank_pid.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::math;
using namespace units::time;
using namespace units::angular_velocity;
using namespace units::velocity;
namespace plt = matplotlibcpp;

template<typename TankPIDType, typename PoseableType>
void
runWheelPID(HID::Joystick &joystick, TankPIDType &robot, PoseableType &poseable)
{
    constexpr meter_t plotLimits = 2_m;

    robot.controlWithThumbsticks(joystick);

    std::cout << "Drive the robot using the two thumbsticks: each stick is for one motor" << std::endl;

    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C

    Pose2<meter_t, radian_t> lastPose;
    std::vector<double> x(1), y(1);
    Stopwatch stopwatch, globalStopwatch;
    stopwatch.start();
    bool driveWithVelocities = false;
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (!pressed) {
            return false;
        }

        switch (button) {
        case HID::JButton::Y:
            if (!driveWithVelocities) {
                robot.stopMoving();
                driveWithVelocities = true;
                std::cout << "Driving using velocities" << std::endl;
                stopwatch.start();
                robot.start();
                globalStopwatch.start();
                robot.tankVelocities(0.1_mps, 0.1_mps);
            }
            return true;
        case HID::JButton::X:
            if (driveWithVelocities) {
                driveWithVelocities = !driveWithVelocities;
                std::cout << "Driving using standard controls" << std::endl;
            }
            return true;
        default:
            break;
        }
        return false;
    });

    std::vector<double> mps, t, targetX, targetY, originY;
    targetX = { 0.0, 1000.0 };
    targetY = { 0.1, 0.1 };
    originY = { 0.0, 0.0 };
    constexpr second_t updateInterval = 200_ms;
    do {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        const auto &currentPose = poseable.getPose();

        plt::figure(1);
        plt::clf();
        plt::subplot(2, 1, 1);
        plotAgent(currentPose, -plotLimits, plotLimits, -plotLimits, plotLimits);
        if (driveWithVelocities && poseable.timeSinceReceived() < 200ms) {
            mps.push_back(robot.updatePose(currentPose, stopwatch.lap()).value());
            t.push_back(static_cast<second_t>(globalStopwatch.elapsed()).value());

            plt::subplot(2, 1, 2);
            plt::plot(t, mps, "b", targetX, targetY, "r--", targetX, originY, "k");
            plt::ylim(0.0, 0.2);
            if (t.back() > 30.0) {
                plt::xlim(t.back() - 30.0, t.back());
            } else {
                plt::xlim(0, 30);
            }
        }

        plt::pause(updateInterval.value());

        lastPose = currentPose;

        // Check for joystick events
        joystick.update();
    } while (plt::fignum_exists(1) && !joystick.isPressed(HID::JButton::B));
    plt::close();
}