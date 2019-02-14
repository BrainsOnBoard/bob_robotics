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
        p.m = tan(angle + 90_deg);
        p.c = pose.y() - p.m * pose.x();
    }
    return p;
}

template<typename PoseType>
auto
calculate(const PoseType &lastPose,
          const PoseType &currentPose,
          const second_t elapsed,
          const meter_t width)
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

    const radian_t currentAngle = atan2(currentPose.y() - centre.y(), currentPose.x() - centre.x());
    const radian_t lastAngle = atan2(lastPose.y() - centre.y(), lastPose.x() - centre.x());
    const radian_t dtheta = circularDistance(currentAngle, lastAngle);
    const radians_per_second_t angularVelocity = dtheta / elapsed;

    const meter_t radius = centre.distance2D(currentPose);

    meters_per_second_t velocity, velocityLeft, velocityRight;
    if (lineLast.m == lineCurrent.m) { // Straight line
        velocity = currentPose.distance2D(lastPose) / elapsed;
        velocityLeft = velocity;
        velocityRight = velocity;
    } else {
        velocity = abs(meters_per_second_t{ radius.value() * angularVelocity.value() });
        velocityLeft = meters_per_second_t{ abs(angularVelocity.value()) * (radius + width / 2).value() };
        velocityRight = meters_per_second_t{ abs(angularVelocity.value()) * (radius - width / 2).value() };
    }

    const radian_t ang = atan2(lastPose.y() - currentPose.y(), lastPose.x() - currentPose.x());
    if (abs(circularDistance(ang, currentPose.yaw())) < 90_deg) {
        velocity = -velocity;
        velocityLeft = -velocityLeft;
        velocityRight = -velocityRight;
    }

    // Return the radius of turning circle
    return std::make_tuple(velocity, velocityLeft, velocityRight, angularVelocity, centre, radius);
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
    Stopwatch stopwatch;
    stopwatch.start();
    do {
        const second_t elapsed = stopwatch.lap();
        const auto &currentPose = robot.getPose();

        plt::clf();
        plotAgent(currentPose, -1.6_m, 1.6_m, -1.6_m, 1.6_m);
        if (lastPose != currentPose) {
            meters_per_second_t velocity, velocityLeft, velocityRight;
            degrees_per_second_t angularVelocity;
            Vector2<meter_t> centre;
            meter_t radius;
            std::tie(velocity, velocityLeft, velocityRight, angularVelocity, centre, radius) = calculate(lastPose, currentPose, elapsed, robot.getRobotWidth() / 2);
            std::cout << centre << " (r = " << radius << ")" << std::endl;
            if (!std::isinf(radius.value())) {
                x[0] = centre.x().value();
                y[0] = centre.y().value();
                plt::plot(x, y, "g+");
            }
            std::stringstream ss;
            ss << "r = " << radius << "; v = " << velocity << "; v_l/r = " << velocityLeft << "/" << velocityRight << "; w = " << angularVelocity;
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
