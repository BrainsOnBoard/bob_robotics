// BoB robotics includes
#include "antworld/agent.h"
#include "antworld/route_continuous.h"
#include "python/python.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cstdlib>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;

int
main(int argc, char **argv)
{
    /*
     * I've set the width of the image to be the same as the (raw) unwrapped
     * images we get from the robot gantry, but the height is greater (cf. 58)
     * because I wanted to keep the aspect ratio as it was (200x40).
     *      -- AD
     */
    const cv::Size RenderSize{ 720, 150 };
    constexpr meter_t AgentHeight = 1_cm;
    constexpr meter_t PathStep = 1_cm;

    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);
    const auto executableDir = filesystem::path(argv[0]).parent_path();

    filesystem::path routePath;
    if (argc > 1)
        routePath = argv[1];
    else {
        // Load default route
        routePath = executableDir / ".." / ".." / "resources" / "antworld" / "ant1_route1.bin";
    }

    // Create route object and load route file specified by command line
    AntWorld::RouteContinuous route(0.2f, 800);
    route.load(routePath.str());

    AntWorld::Renderer renderer{ 256, 0.001, 1000.0, 360_deg };
    renderer.getWorld().load(executableDir / ".." / ".." / "resources" / "antworld" / "world5000_gray.bin",
                             { 0.0f, 1.0f, 0.0f },
                             { 0.898f, 0.718f, 0.353f });
    AntWorld::AntAgent agent{ *window, renderer, RenderSize };

    agent.setPosition(0_m, 0_m, AgentHeight);

    // Look for python modules in current dir
    Python::appendToPythonPath(executableDir);
    Python::importModule("ant_world_python");

    cv::Mat im{ RenderSize, CV_8UC3 }; // To hold ant's view
    wrappy::NumpyArray npArray{ im }; // This now points to the data in im

    for (auto distance = 0_m; distance < route.getLength(); distance += PathStep) {
        // Move agent along route
        Pose3<meter_t, degree_t> pose = route.getPose(distance);
        pose.z() = AgentHeight;
        agent.setPose(pose);

        // Read frame
        agent.readFrame(im);
        Python::call("ant_world_python.my_function", npArray);
    }
}
