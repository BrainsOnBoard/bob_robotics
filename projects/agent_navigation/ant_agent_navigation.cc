#include "navigation_common.h"

// BoB robotics includes
#include "common/main.h"
#include "libantworld/agent.h"

// OpenCV
#include <opencv2/opencv.hpp>

using namespace BoBRobotics;

int
bob_main(int, char **)
{
    const cv::Size RenderSize{ 720, 150 };
    const meter_t AntHeight = 1_cm;
    const float ForwardSpeed = 1.f;

    // Initialise OpenGL
    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    auto &world = renderer.getWorld();
    world.load("../../libantworld/world5000_gray.bin",
               { 0.0f, 1.0f, 0.0f },
               { 0.898f, 0.718f, 0.353f });
    const auto minBound = world.getMinBound();
    const auto maxBound = world.getMaxBound();

    // Create agent and put in the centre of the world
    AntWorld::AntAgent ant(window.get(), renderer, RenderSize, 1_mps);
    ant.setPosition((maxBound[0] - minBound[0]) / 2, (maxBound[1] - minBound[1]) / 2, AntHeight);

    // Run navigation
    runNavigation(ant, ant, ForwardSpeed, ant, minBound[0], maxBound[0], minBound[1], maxBound[1]);

    return EXIT_SUCCESS;
}
