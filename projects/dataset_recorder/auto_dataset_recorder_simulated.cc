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

void plotArrow(const Pose3<meter_t, radian_t> &pose)
{
    static const std::map<std::string, std::string> kwargs = { std::make_pair("angles", "xy") };

    const std::vector<double> x = { pose.x().value() };
    const std::vector<double> y = { pose.y().value() };
    const std::vector<double> u = { cos(pose.yaw()) };
    const std::vector<double> v = { sin(pose.yaw()) };

    plt::quiver(x, y, u, v, kwargs);
}

int
bobMain(int argc, char **argv)
{
    constexpr meters_per_second_t MaxSpeed = 1.4_mps; // car's max speed
    constexpr degree_t MaxTurn = 30_deg;              // car's maximum turning angle
    constexpr millimeter_t LookAheadDistance = 1_m;   // lookahead distance
    constexpr millimeter_t StoppingDist = 5_cm;       // car's stopping distance

    BOB_ASSERT(argc == 2);

    Robots::Ackermann::SimulatedAckermann robot{ MaxSpeed, 500_mm, 0_m, MaxTurn };
    AutoController controller{ argv[1], LookAheadDistance, robot.getDistanceBetweenAxes(), StoppingDist };
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
    Video::RandomInput<> cam{ { 360, 100 } };
    cam.setFrameRate(20_Hz);
    auto recorder = database.createVideoRouteRecorder(cam.getOutputSize(),
                                                      cam.getFrameRate(),
                                                      { "UTM zone" });

    cv::Mat fr;
    do {
        plt::figure(1);
        plt::clf();
        plotArrow(robot.getPose());
        plt::plot(x, y);
        plt::axis("equal");
        plt::pause((1 / cam.getFrameRate()).value());

        if (!controller.step(robot.getPose(), robot, MaxSpeed, MaxTurn)) {
            break;
        }

        // Also save UTM zone
        cam.readFrameSync(fr);
        recorder->record(pose, fr, route[0].zone);
    } while (plt::fignum_exists(1));

    return EXIT_SUCCESS;
}
