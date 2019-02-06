// BoB robotics includes
#include "robots/tank_pid.h"
#include "common/main.h"
#include "common/pose.h"
#include "hid/joystick.h"
#include "robots/car_display.h"
#include "robots/simulated_tank.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::length;

int
bob_main(int, char **)
{
    Robots::SimulatedTank<> robot(0.3_mps, 104_mm); // Tank agent
    Robots::CarDisplay display;                     // For displaying the agent
    Robots::TankPID pid(robot, .1f, .1f, .1f, 0.5f);       // PID controller

    HID::Joystick joystick(0.25f);
    robot.controlWithThumbsticks(joystick);

    const Vector2<millimeter_t> goal{}; // Goal is origin
    bool pidRunning = false;
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (!pressed) {
            return false;
        }

        switch (button) {
        case HID::JButton::Start:
            robot.setPose({}); // Reset to origin
            return true;
        case HID::JButton::Y: // Toggle PID control
            pidRunning = !pidRunning;
            if (pidRunning) {
                std::cout << "PID control started" << std::endl;
                pid.start(goal);
            } else {
                std::cout << "PID control stopped" << std::endl;
                robot.stopMoving();
            }
            return true;
        default:
            return false;
        }
    });

    std::cout << "Drive the car using the two thumbsticks: each stick is for one motor" << std::endl;

    do {
        const auto robotPose = robot.getPose();

        // Refresh display
        display.runGUI(robotPose);

        // Run PID controller
        if (pidRunning) {
            pid.update(robotPose);
        }

        // Check for joystick events
        joystick.update();

        // A small delay so we don't hog CPU
        std::this_thread::sleep_for(5ms);
    } while (!joystick.isPressed(HID::JButton::B) && display.isOpen());

    return EXIT_SUCCESS;
}
