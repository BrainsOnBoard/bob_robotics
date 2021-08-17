#include "auto_controller.h"

// BoB robotics includes
#include "common/macros.h"
#include "navigation/image_database.h"
#include "robots/ackermann/simulated_ackermann.h"
#include "video/randominput.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"

// Standard C++ includes
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

    BOB_ASSERT(argc == 2);

    Robots::Ackermann::SimulatedAckermann robot{ MaxSpeed, 500_mm, 0_m, MaxTurn };
    RobotArrow arrow;
    AutoController controller{ argv[1], LookAheadDistance, robot.getDistanceBetweenAxis(), StoppingDist };
    Navigation::ImageDatabase database;
    const auto &route = controller.getRoute();

    std::vector<double> x, y;
    for (const auto &utm : route) {
        x.push_back(utm.easting.value());
        y.push_back(utm.northing.value());

    }

    Pose2<meter_t, degree_t> pose{ route[0].easting + 0.5_m, route[0].northing + 0.5_m, 270_deg };
    robot.setPose(pose);

    // Record random images into database (as this is just for testing)
    constexpr auto frameRate = 20_Hz;
    Video::RandomInput<> cam{ { 360, 100 } };
    auto recorder = database.getRouteVideoRecorder(cam.getOutputSize(), frameRate, { "UTM zone" });

    cv::Mat fr;
    do {
        plt::figure(1);
        plt::clf();
        arrow.plot(robot.getPose());
        plt::plot(x, y);
        plt::axis("equal");
        plt::pause((1 / frameRate).value());

        if (!controller.step(robot.getPose(), robot, MaxSpeed, MaxTurn)) {
            break;
        }

        // Also save UTM zone
        cam.readFrameSync(fr);
        recorder.record(pose, pose.yaw(), fr, route[0].zone);
    } while (plt::fignum_exists(1));

    return EXIT_SUCCESS;
}
