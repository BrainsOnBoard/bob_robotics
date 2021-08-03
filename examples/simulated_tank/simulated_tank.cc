// BoB robotics includes
#include "plog/Log.h"
#include "robots/tank/simulated_tank.h"
#include "viz/sfml/joystick_keyboard.h"
#include "viz/sfml/sfml_world.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::literals;

int bobMain(int, char **)
{
    Robots::Tank::SimulatedTank<> robot(0.3_mps, 104_mm); // Tank agent
    Viz::SFMLWorld display;                         // For displaying the agent
    auto car = display.createCarAgent();

    auto joystick = Viz::JoystickKeyboard::createJoystick();
    robot.controlWithThumbsticks(*joystick);

    joystick->addHandler([&robot](auto &, HID::JButton button, bool pressed) {
        if (pressed && button == HID::JButton::Start) {
            robot.setPose({}); // Reset to origin
            return true;
        } else {
            return false;
        }
    });

    LOGI << "Drive the car using the two thumbsticks: each stick is for one motor";

    do {
        // Refresh display
        car.setPose(robot.getPose());
        display.update(car);

        // Check for joystick events
        joystick->update();
    } while (!joystick->isPressed(HID::JButton::B) && display.isOpen());

    return EXIT_SUCCESS;
}
