// BoB robotics includes
#include "common/assert.h"
#include "common/circstat.h"
#include "common/main.h"
#include "common/plot_agent.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "robots/simulated_tank.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::length;
namespace plt = matplotlibcpp;

template<typename PoseType>
auto
getStraightLineEquation(const PoseType &pose)
{
    struct Params
    {
        double m;
        typename PoseType::LengthType c;

        bool isVertical() const
        {
            return std::isinf(m);
        }
    } p;

    const auto angle = normaliseAngle180(pose.yaw());
    if (angle == 0_deg || angle == 180_deg) {
        p.m = std::numeric_limits<double>::infinity();
        p.c = typename PoseType::LengthType{ std::numeric_limits<double>::signaling_NaN() };
    } else {
        p.m = units::math::tan(angle + 90_deg);
        p.c = pose.y() - p.m * pose.x();
    }
    return p;
}

template<typename PoseType>
auto
calculate(const PoseType &lastPose, const PoseType &currentPose)
{
    const auto lineLast = getStraightLineEquation(lastPose);
    const auto lineCurrent = getStraightLineEquation(currentPose);
    Vector2<meter_t> centre;
    if (lineLast.isVertical()) {
        centre.x() = lastPose.x();
        centre.y() = lineCurrent.m * centre.x() + lineCurrent.c;
    } else if (lineCurrent.isVertical()) {
        centre.x() = currentPose.x();
        centre.y() = lineLast.m * centre.x() + lineLast.c;
    } else {
        centre.x() = (lineCurrent.c - lineLast.c) / (lineLast.m - lineCurrent.m);
        centre.y() = lineLast.m * centre.x() + lineLast.c;
    }

    // Return the radius of turning circle
    return std::make_pair(centre, centre.distance2D(currentPose));
}

int
bob_main(int, char **)
{
    Robots::SimulatedTank<> robot(0.3_mps, 104_mm); // Tank agent

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
    do {
        const auto &currentPose = robot.getPose();

        plt::clf();
        plotAgent(currentPose, -1.6_m, 1.6_m, -1.6_m, 1.6_m);
        if (lastPose != currentPose) {
            const auto pos = calculate(lastPose, currentPose);
            std::cout << pos.first << " (r = " << pos.second << ")" << std::endl;
            if (!std::isinf(pos.second.value())) {
                x[0] = pos.first.x().value();
                y[0] = pos.first.y().value();
                plt::plot(x, y, "g+");
            }
            std::stringstream ss;
            ss << "Radius = " << pos.second;
            plt::title(ss.str());
        }
        plt::pause(0.05);

        lastPose = currentPose;

        // Check for joystick events
        joystick.update();

    } while (!joystick.isPressed(HID::JButton::B) && plt::fignum_exists(1));
    plt::close();

    return EXIT_SUCCESS;
}
