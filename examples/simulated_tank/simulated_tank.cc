// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
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

int
bob_main(int, char **)
{
    Robots::SimulatedTank<> robot(0.3_mps, 104_mm); // Tank agent
    Viz::SFMLWorld display;                            // For displaying the agent
    auto car = display.createCarAgent();

    HID::Joystick joystick(0.25f);
    robot.controlWithThumbsticks(joystick);

    joystick.addHandler([&robot](HID::JButton button, bool pressed) {
        if (pressed && button == HID::JButton::Start) {
            robot.setPose({}); // Reset to origin
            return true;
        } else {
            return false;
        }
    });

    std::cout << "Drive the car using the two thumbsticks: each stick is for one motor" << std::endl;

    do {
        // Refresh display
        car.setPose(robot.getPose());
        display.update(car);

        // Check for joystick events
        if (!joystick.update()) {
            // A small delay so we don't hog CPU
            std::this_thread::sleep_for(5ms);
        }
    } while (!joystick.isPressed(HID::JButton::B) && display.isOpen());

    return EXIT_SUCCESS;
}
