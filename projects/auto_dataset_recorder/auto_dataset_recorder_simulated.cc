#include "common.h"

// BoB robotics includes
#include "common/macros.h"
#include "common/map_coordinate.h"
#include "navigation/image_database.h"
#include "robots/ackermann/simulated_ackermann.h"
#include "robots/control/pure_pursuit_controller.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>


using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::velocity;
using namespace units::literals;
using namespace units::math;
namespace plt = matplotlibcpp;

class RobotArrow
{
public:
    void plot(const Pose3<meter_t, radian_t> &pose)
    {
        m_X[0] = pose.x().value();
        m_Y[0] = pose.y().value();
        m_U[0] = cos(pose.yaw());
        m_V[0] = sin(pose.yaw());

        plt::quiver(m_X, m_Y, m_U, m_V, m_Kwargs);
    }

private:
    std::vector<double> m_X = { 0 }, m_Y = { 0 }, m_U = { 0 }, m_V = { 0 };
    const std::map<std::string, std::string> m_Kwargs = { std::make_pair("angles", "xy") };
};

int
bobMain(int argc, char **argv)
{
    constexpr meters_per_second_t MaxSpeed = 1.4_mps; // car's max speed
    constexpr degree_t MaxTurn = 30_deg;              // car's maximum turning angle
    constexpr millimeter_t LookAheadDistance = 1_m;   // lookahead distance
    constexpr millimeter_t StoppingDist = 5_cm;       // car's stopping distance

    BOB_ASSERT(argc > 1);
    const auto route = loadRoute(argv[1]);

    Robots::Ackermann::SimulatedAckermann robot{ MaxSpeed, 500_mm, 0_m, MaxTurn };
    RobotArrow arrow;
    Robots::PurePursuitController controller{ LookAheadDistance, robot.getDistanceBetweenAxis(), StoppingDist };

    constexpr meter_t infM{ std::numeric_limits<double>::infinity() };
    auto xLims = std::make_pair(infM, -infM);
    auto yLims = xLims;
    std::vector<double> x, y;
    for (const auto &utm : route) {
        x.push_back(utm.easting.value());
        y.push_back(utm.northing.value());

        xLims.first = std::min(xLims.first, utm.easting);
        xLims.second = std::max(xLims.second, utm.easting);
        yLims.first = std::min(yLims.first, utm.northing);
        yLims.second = std::max(yLims.second, utm.northing);

        controller.addWayPoint(utm.toVector());
    }

    std::cout << "x limits: [" << xLims.first << ", " << xLims.second << "]\n";
    std::cout << "y limits: [" << yLims.first << ", " << yLims.second << "]\n";

    Pose2<meter_t, degree_t> pose{ route[0].easting + 0.5_m, route[0].northing + 0.5_m, 270_deg };
    robot.setPose(pose);

    do {
        plt::figure(1);
        plt::clf();
        arrow.plot(robot.getPose());
        plt::plot(x, y);
        plt::axis("equal");
        plt::pause(0.05);

        const auto lookPoint = controller.getLookAheadPoint(robot.getPose());
        if (!lookPoint) {
            LOGE << "Robot got stuck! 😭";
            break;
        }

        // calculate turning angle with controller
        const auto turningAngle = controller.getTurningAngle(robot.getPose(), lookPoint);
        if (turningAngle) {
            const auto ang = turningAngle.value();
            if (abs(ang) > MaxTurn) {
                robot.move(MaxSpeed, copysign(MaxTurn, ang));
            } else {
                robot.move(MaxSpeed, ang);
            }
        } else {
            LOGI << "Reached destination!";
            break;
        }
    } while (plt::fignum_exists(1));

    return EXIT_SUCCESS;
}
