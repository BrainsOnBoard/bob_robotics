// BoB robotics includes
#include "antworld/agent.h"
#include "common/logging.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <chrono>
#include <thread>
#include <tuple>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::angle;
using namespace units::length;

int
bob_main(int, char **argv)
{
    const cv::Size RenderSize{ 720, 150 };
    const meter_t AntHeight = 1_cm;

    HID::Joystick joystick;

    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    auto &world = renderer.getWorld();
    world.load(filesystem::path(argv[0]).parent_path() / "../../resources/antworld/world5000_gray.bin",
               { 0.0f, 1.0f, 0.0f },
               { 0.898f, 0.718f, 0.353f });
    const auto minBound = world.getMinBound();
    const auto maxBound = world.getMaxBound();

    // Create agent and put in the centre of the world
    AntWorld::AntAgent agent(*window, renderer, RenderSize);
    agent.setPosition((maxBound[0] - minBound[0]) / 2, (maxBound[1] - minBound[1]) / 2, AntHeight);

    // Control the agent with a joystick
    agent.addJoystick(joystick);

    LOG_INFO << "Press the B button to quit";
    Pose3<meter_t, degree_t> lastPose;
    Stopwatch stopwatch;
    stopwatch.start();
    while (agent.isOpen() && !joystick.isDown(HID::JButton::B)) {
        /*
         * We need to recalculate pose *before* checking for joystick events,
         * so that if the agent's velocity changes, it is starting from the
         * correct point.
         */
        agent.updatePose(stopwatch.lap());

        // Poll joystick
        joystick.update();

        const auto pose = agent.getPose<meter_t, degree_t>();
        if (pose == lastPose) {
            std::this_thread::sleep_for(5ms);
            continue;
        }

        // LOG_INFO << "Pose: " << pose;
        lastPose = pose;

        // Update display
        agent.update();
    }

    return EXIT_SUCCESS;
}
