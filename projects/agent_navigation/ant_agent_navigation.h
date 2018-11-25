#pragma once

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/pose.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "navigation/perfect_memory.h"
#include "robots/robot.h"
#include "video/input.h"
#include "video/unwrapped_input.h"
#include "viz/agent_renderer.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::time;
using namespace std::literals;

using ObjectType = std::pair<std::vector<millimeter_t>, std::vector<millimeter_t>>;

class EggTimer
{
private:
    using TimeType = std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds>;
    TimeType m_StartTime;
    std::chrono::nanoseconds m_Duration = std::chrono::nanoseconds::zero();

public:
    // void start(const nanosecond_t duration)
    // {
    //     start(std::chrono::nanoseconds(static_cast<int64_t>(duration.value())));
    // }

    void start(const std::chrono::nanoseconds duration)
    {
        m_StartTime = now();
        m_Duration = duration;
    }

    bool running() const
    {
        return m_Duration != std::chrono::nanoseconds::zero();
    }

    void stop()
    {
        m_Duration = std::chrono::nanoseconds::zero();
    }

    bool finished() const
    {
        return (now() - m_StartTime) >= m_Duration;
    }

    static TimeType now()
    {
        return std::chrono::high_resolution_clock::now();
    }
}; // EggTimer

filesystem::path
getRoutePath(const int routeNum)
{
    const filesystem::path routeBasePath = "routes";
    return routeBasePath / ("route" + std::to_string(routeNum));
}

class TrainingDatabase
  : Navigation::ImageDatabase
{
public:
    TrainingDatabase(const int routeNum, const Video::Input &videoInput)
      : Navigation::ImageDatabase(getRoutePath(routeNum))
      , m_Recorder(*this)
    {
        auto &metadata = m_Recorder.getMetadataWriter();
        metadata << "camera" << videoInput
                 << "needsUnwrapping" << false
                 << "isGreyscale" << true;

        auto unwrappedInput = dynamic_cast<const Video::UnwrappedInput *>(&videoInput);
        if (unwrappedInput) {
            metadata << "unwrapper" << unwrappedInput->getUnwrapper();
        }
    }

    ~TrainingDatabase()
    {
        m_Recorder.save();
        std::cout << "Stopping training (" << size() << " stored)" << std::endl;
    }

    Navigation::ImageDatabase::RouteRecorder &getRouteRecorder()
    {
        return m_Recorder;
    }

private:
    Navigation::ImageDatabase::RouteRecorder m_Recorder;
};

template<typename LengthUnit, typename PoseGetterType, typename DisplayType>
void
runNavigation(Robots::Robot &robot,
              PoseGetterType &poseGetter,
              const float forwardSpeed,
              const float turnSpeed,
              Video::Input &videoInput,
              const Position2<LengthUnit> &minBounds,
              const Position2<LengthUnit> &maxBounds,
              DisplayType &display)
{
    const auto robotTurnSpeed = robot.getMaximumTurnSpeed();

    const filesystem::path routeBasePath = "routes";
    filesystem::create_directory(routeBasePath);

    // Count number of routes
    int numRoutes = 0;
    while (getRoutePath(++numRoutes).exists())
        ;
    std::unique_ptr<TrainingDatabase> trainingDatabase;

    Navigation::PerfectMemoryRotater<> pm(videoInput.getOutputSize());

    auto &catcher = BackgroundExceptionCatcher::getInstance();
    catcher.trapSignals();

    // Control robot with joystick
    HID::Joystick joystick;
    robot.addJoystick(joystick);
    std::cout << "Joystick opened" << std::endl;

    bool testing = false;
    EggTimer turnTimer;
    Viz::AgentRenderer<LengthUnit> renderer(10_cm, minBounds, maxBounds);
    auto trainingLine = renderer.createLine(sf::Color::Blue);
    auto testingLine = renderer.createLine(sf::Color::Green);
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (pressed) {
            return false;
        }

        switch (button) {
        case HID::JButton::Y:
            BOB_ASSERT(!testing);

            if (trainingDatabase) {
                trainingDatabase.reset();
            } else {
                trainingDatabase = std::make_unique<TrainingDatabase>(numRoutes++, videoInput);
                trainingLine.clear();
                std::cout << "Recording training images" << std::endl;
            }
            return true;
        case HID::JButton::X:
            if (testing) {
                robot.stopMoving();
                testing = false;
                turnTimer.stop();
                std::cout << "Stopping testing" << std::endl;
            } else {
                if (trainingDatabase) {
                    trainingDatabase.reset();
                }

                std::cout << "Starting testing" << std::endl;
                testingLine.clear();
                if (pm.getNumSnapshots() == 0) {
                    const Navigation::ImageDatabase database(getRoutePath(numRoutes - 1));
                    pm.trainRoute(database, /*imageStep=*/10);
                    std::cout << pm.getNumSnapshots() << " snapshots loaded." << std::endl;
                }
                testing = true;

                robot.moveForward(forwardSpeed);
            }
            return true;
        default:
            return !testing;
        }
    });

    cv::Mat frame;
    do {
        catcher.check();

        // if (trainingDatabase) {
        //     plotter.setTitle("Training");
        // } else if (testing) {
        //     plotter.setTitle("Testing");
        // }

        joystick.update();
        if (turnTimer.running()) {
            if (turnTimer.finished()) {
                robot.moveForward(forwardSpeed);
                turnTimer.stop();
            }
        }

        renderer.update(poseGetter.getPose(), { trainingLine, testingLine });
        display.update();
        if (videoInput.readGreyscaleFrame(frame)) {
            const auto pose = poseGetter.template getPose<millimeter_t>();
            if (trainingDatabase) {
                trainingLine.append(pose);
                trainingDatabase->getRouteRecorder().record(pose.position(), pose.yaw(), frame);
            } else if (testing) {
                Timer<> t{ "Time to calculate: " };
                const degree_t heading = std::get<0>(pm.getHeading(frame));
                std::cout << "Heading: " << heading << std::endl;

                turnTimer.start(heading / robotTurnSpeed);
                robot.turnOnTheSpot(heading < 0_deg ? -turnSpeed : turnSpeed);

                testingLine.append(pose);
            }
        }

    } while (!joystick.isPressed(HID::JButton::B) && display.isOpen() && renderer.isOpen());
}
