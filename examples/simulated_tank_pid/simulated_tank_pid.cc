// BoB robotics includes
#include "common/main.h"
#include "common/pose.h"
#include "hid/joystick.h"
#include "robots/control/tank_pid.h"
#include "robots/simulated_tank.h"
#include "viz/sfml_world/sfml_world.h"

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
    Robots::SimulatedTank<> robot(0.3_mps, 104_mm);                // Tank agent
    Viz::SFMLWorld display;                                        // For displaying the agent
    auto pid = Robots::createTankPID(robot, robot, .1f, .1f, .1f); // PID controller
    auto car = display.createCarAgent();

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
                pid.moveTo(goal);
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

        // Check for joystick events
        joystick.update();

        // Refresh display
        car.setPose(robot.getPose());
        display.update(car);

        // Run PID controller
        if (pidRunning && !pid.pollPositioner()) {
            // Stop PID if we're at goal
            pidRunning = false;
        }

        // A small delay so we don't hog CPU
        std::this_thread::sleep_for(5ms);
    } while (!joystick.isPressed(HID::JButton::B) && display.isOpen());

    return EXIT_SUCCESS;
}
