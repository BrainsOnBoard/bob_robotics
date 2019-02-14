// BoB robotics includes
#include "common/assert.h"
#include "common/circstat.h"
#include "common/main.h"
#include "common/plot_agent.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/simulated_tank.h"
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
using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using namespace units::math;
using namespace units::time;
using namespace units::angular_velocity;
using namespace units::velocity;
namespace plt = matplotlibcpp;

int
bob_main(int, char **)
{
    Robots::TankPID<Robots::SimulatedTank<>> robot(1.f, 0.f, 0.f, 0.3_mps, 104_mm);
    robot.updatePose(Pose2<meter_t, radian_t>{}, 0_s);

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

    decltype(robot)::PoseType lastPose;
    std::vector<double> x(1), y(1);
    Stopwatch stopwatch;
    stopwatch.start();
    do {
        const auto &currentPose = robot.getPose();

        plt::figure(1);
        plt::clf();
        plotAgent(currentPose, -1.6_m, 1.6_m, -1.6_m, 1.6_m);
        if (lastPose != currentPose) {
            robot.updatePose(currentPose, stopwatch.lap());
        }
        plt::pause(0.1);

        lastPose = currentPose;

        // Check for joystick events
        joystick.update();

    } while (plt::fignum_exists(1));
    plt::close();

    return EXIT_SUCCESS;
}
