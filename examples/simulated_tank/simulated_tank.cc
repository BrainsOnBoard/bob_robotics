// BoB robotics includes
#include "common/main.h"
#include "common/plot_agent.h"
#include "hid/joystick.h"
#include "robots/simulated_tank.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::literals;

int
bob_main(int, char **)
{
    Robots::SimulatedTank<units::length::millimeter_t, units::angle::radian_t> robot(0.1_mps, 34_mm);
    HID::Joystick joystick;
    robot.addJoystick(joystick);

    do {
        joystick.update();
        plotAgent(robot, { -2000, 2000 }, { -2000, 2000 });
        matplotlibcpp::pause(0.1);
    } while (!joystick.isPressed(HID::JButton::B));

    matplotlibcpp::close();

    return EXIT_SUCCESS;
}
