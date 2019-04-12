// BoB robotics includes
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "antworld/agent.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>
#include <tuple>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::length;
using namespace units::angle;

int
main()
{
    const cv::Size RenderSize{ 720, 150 };
    const meter_t AntHeight = 1_cm;

    HID::Joystick joystick;

    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    auto &world = renderer.getWorld();
    world.load("../../include/antworld/world5000_gray.bin",
               { 0.0f, 1.0f, 0.0f },
               { 0.898f, 0.718f, 0.353f });
    const auto minBound = world.getMinBound();
    const auto maxBound = world.getMaxBound();

    // Create agent and put in the centre of the world
    AntWorld::AntAgent agent(window.get(), renderer, RenderSize);
    agent.setPosition((maxBound[0] - minBound[0]) / 2, (maxBound[1] - minBound[1]) / 2, AntHeight);

    // Control the agent with a joystick
    agent.addJoystick(joystick);

    std::cout << "Press the B button to quit" << std::endl;
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

        auto pose = agent.getPose<meter_t, degree_t>();
        if (pose == lastPose) {
            std::this_thread::sleep_for(5ms);
            continue;
        }

        // std::cout << "Pose: " << pose.x() << ", " << pose.y() << ", " << pose.yaw() << std::endl;
        lastPose = pose;

        // Update display
        agent.update();
    }
}
