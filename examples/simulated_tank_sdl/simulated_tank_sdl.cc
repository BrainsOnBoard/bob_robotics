// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/simulated_tank.h"
#include "robots/simulator.h"

// Third-party includes
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::literals;

int
bob_main(int, char **)
{
    Robots::Simulator robot(50_cm, 0.1_mps, 34_mm);
    HID::Joystick joystick(0.25f);
    robot.controlWithThumbsticks(joystick);

    joystick.addHandler([&robot](HID::JButton button, bool pressed) {
        if (pressed && button == HID::JButton::Start) {
            robot.setPose({ 0_m, 0_m, 0_rad });
            return true;
        } else {
            return false;
        }
    });

    // robot.drive(0.1_mps, 50_deg_per_s);
    do {
        joystick.update();
        robot.simulationStep();
    } while (!joystick.isPressed(HID::JButton::B) && !robot.didQuit());

    return EXIT_SUCCESS;
}
