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
namespace plt = matplotlibcpp;

int
bob_main(int, char **)
{
    Robots::SimulatedTank<units::length::millimeter_t, units::angle::radian_t> robot(0.1_mps, 34_mm);
    HID::Joystick joystick(0.25f);

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

        robot.tank(-joystick.getState(HID::JAxis::LeftStickVertical),
                   -joystick.getState(HID::JAxis::RightStickVertical));

        constexpr int lim = 250;
        plotAgent(robot, { -lim, lim }, { -lim, lim });
        plt::pause(0.1);
    } while (!joystick.isPressed(HID::JButton::B) && plt::fignum_exists(1));

    plt::close();

    return EXIT_SUCCESS;
}
