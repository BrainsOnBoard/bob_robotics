// BoB robotics includes
#include "libantworld/agent.h"
#include "libantworld/route_continuous.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// GLFW
#include <GLFW/glfw3.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <iostream>
#include <tuple>
#include <vector>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;

int main(int argc, char **argv)
{
    filesystem::path routePath;
    if (argc == 1) {
        routePath = filesystem::path(argv[0]).parent_path() / ".." / ".." / "libantworld" / "ant1_route1.bin";
    } else {
        routePath = filesystem::path(argv[1]);
    }

    const cv::Size renderSize{ 720, 150 };
    const auto window = AntWorld::AntAgent::initialiseWindow(renderSize);
    constexpr meter_t routeSep = 10_cm;
    constexpr meter_t height = 10_cm;

    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    renderer.getWorld().load("../../libantworld/world5000_gray.bin",
                             {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});
    AntWorld::AntAgent agent(window.get(), renderer, renderSize.width, renderSize.height);
    AntWorld::RouteContinuous route(0.2f, 800, routePath.str());

    std::cout << "Getting training route..." << std::endl;
    std::vector<cv::Mat> trainingRoute;
    for (meter_t dist = 0_m; dist < route.getLength() && !glfwWindowShouldClose(window.get()); dist += routeSep) {
        meter_t x, y;
        degree_t theta;
        std::tie(x, y, theta) = route.getPosition(dist);
        agent.setPosition(x, y, height);
        agent.setAttitude(theta, 0_deg, 0_deg);

        trainingRoute.emplace_back(renderSize, CV_32F);
        agent.readGreyscaleFrame(trainingRoute.back());
    }
}
