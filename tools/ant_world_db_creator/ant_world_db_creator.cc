
// BoB robotics includes
#include "common/pose.h"
#include "libantworld/agent.h"
#include "libantworld/common.h"
#include "libantworld/renderer.h"
#include "libantworld/route_continuous.h"
#include "navigation/image_database.h"
#include "video/opengl.h"

// Third-party includes
#include "third_party/path.h"

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;

/*
* I've set the width of the image to be the same as the (raw) unwrapped
* images we get from the robot gantry, but the height is greater (cf. 58)
* because I wanted to keep the aspect ratio as it was (200x40).
*      -- AD
*/
const cv::Size RenderSize{ 720, 150 };
constexpr millimeter_t AgentHeight = 1_cm;

int
main(int argc, char **argv)
{
    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    AntWorld::AntAgent agent(window.get(), renderer, RenderSize);

    // Load world
    renderer.getWorld().load("../../libantworld/world5000_gray.bin",
                             { 0.0f, 1.0f, 0.0f },
                             { 0.898f, 0.718f, 0.353f });

    const auto checkWindow = [&window]() {
        return !glfwWindowShouldClose(window.get());
    };

    if (argc > 1) {
        // Create route object and load route file specified by command line
        AntWorld::RouteContinuous route(0.2f, 800);
        route.load(argv[1]);

        // Get filename from route path
        std::string databaseName = filesystem::path(argv[1]).filename();

        // If it exists, remove extension
        const size_t pos = databaseName.find_last_of(".");
        if (pos != std::string::npos) {
            databaseName = databaseName.substr(0, pos);
        }

        constexpr millimeter_t pathStep = 1_cm;

        // Make vector of agent's poses
        std::vector<Pose3<millimeter_t, degree_t>> poses;
        for (auto distance = 0_mm; distance < route.getLength(); distance += pathStep) {
            Pose3<millimeter_t, degree_t> pose = route.getPose(distance);
            pose.z() = AgentHeight;
            poses.push_back(pose);
        }

        Navigation::ImageDatabase database(databaseName);
        auto routeRecorder = database.getRouteRecorder();
        routeRecorder.addMetadata(agent, false, false);

        // Save images
        routeRecorder.run(agent, agent, poses, checkWindow);
    } else {
        Navigation::ImageDatabase database("world5000_grid");

        const millimeter_t gridSpacing = 10_cm;

        // Get world bounds
        const auto &worldMinBound = renderer.getWorld().getMinBound();
        const auto &worldMaxBound = renderer.getWorld().getMaxBound();

        // Make GridRecorder
        Range xrange({ worldMinBound[0], worldMaxBound[0] }, gridSpacing);
        Range yrange({ worldMinBound[1], worldMaxBound[1] }, gridSpacing);
        auto gridRecorder = database.getGridRecorder(xrange, yrange, AgentHeight);
        gridRecorder.addMetadata(agent, false, false);

        // Save images
        gridRecorder.run(agent, agent, checkWindow);
    }
}
