#include "ant_agent_navigation.h"

// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
#include "libantworld/agent.h"
#include "viz/agent_renderer.h"

// Third-party includes
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace std::literals;

int
bob_main(int, char **)
{
    const cv::Size RenderSize{ 360, 75 };
    const meter_t AntHeight = 1_cm;
    const float ForwardSpeed = 1.f;
    const float TurnSpeed = 0.5f;

    // Initialise OpenGL
    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    auto &world = renderer.getWorld();
    const std::vector<GLfloat> objectsGL = world.load("../../libantworld/world5000_gray.bin",
                                                      { 0.0f, 1.0f, 0.0f },
                                                      { 0.898f, 0.718f, 0.353f });
    const auto minBound = world.getMinBound();
    const auto maxBound = world.getMaxBound();

    // Get objects
    std::vector<std::vector<Position2<meter_t>>> objects;
    objects.reserve(objectsGL.size() / (3 * 3)); // Number of triangles
    for (auto it = objectsGL.begin() + 18; it < objectsGL.end(); it += 3 * 3) {
        objects.emplace_back(3);
        auto &object = objects.back();
        for (size_t c = 0; c < 2; c++) {
            for (size_t v = 0; v < 3; v++) {
                object[v][c] = meter_t(*(it + (3 * v) + c));
            }
        }
    }

    // Create agent and put in the centre of the world
    AntWorld::AntAgent ant(window.get(), renderer, RenderSize, 0.25_mps, 50_deg_per_s);
    ant.setPosition(0_m, 0_m, AntHeight);
    auto resetPosition = [&ant, AntHeight]() {
        ant.stopMoving();
        ant.setPosition(0_m, 0_m, AntHeight);
        ant.setAttitude(0_deg, 0_deg, 0_deg);
    };

    runNavigation<meter_t>(ant, ant, ForwardSpeed, TurnSpeed, ant, minBound, maxBound, ant, resetPosition, objects);

    return EXIT_SUCCESS;
}
