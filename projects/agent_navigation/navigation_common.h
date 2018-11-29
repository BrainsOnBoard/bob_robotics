#pragma once

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/pose.h"
#include "common/stopwatch.h"
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
#include <functional>
#include <iostream>
#include <memory>
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
              DisplayType &display,
              std::function<void()> resetPosition = nullptr,
              const std::vector<std::vector<Position2<LengthUnit>>> &objects = {},
              units::angular_velocity::radians_per_second_t robotTurnSpeed = 0_deg_per_s)
{
    if (robotTurnSpeed == 0_deg_per_s) {
        robotTurnSpeed = robot.getMaximumTurnSpeed();
    }
    std::cout << "Maximum turn speed: " << robotTurnSpeed << std::endl;

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

    bool testing = false;
    EggTimer turnTimer;
    Viz::AgentRenderer<LengthUnit> renderer(10_cm, minBounds, maxBounds);
    renderer.addObjects(objects);
    auto trainingLine = renderer.createLine(sf::Color::Blue);
    auto testingLine = renderer.createLine(sf::Color::Green);

    // Control robot with joystick
    std::unique_ptr<HID::Joystick> joystick;
    try {
        joystick = std::make_unique<HID::Joystick>();
    } catch (std::runtime_error &) {
        std::cerr << "Warning: Could not open joystick" << std::endl;
    }
    if (joystick) {
        robot.addJoystick(*joystick);
        std::cout << "Joystick opened" << std::endl;

        joystick->addHandler([&](HID::JButton button, bool pressed) {
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
                        const size_t imageStep = 10;
                        trainingLine.clear();
                        for (auto dbEntry = database.begin(); dbEntry < database.end(); dbEntry += imageStep) {
                            trainingLine.append(dbEntry->position);
                            pm.train(dbEntry->loadGreyscale());
                        }
                        std::cout << pm.getNumSnapshots() << " snapshots loaded." << std::endl;
                    }
                    testing = true;

                    robot.moveForward(forwardSpeed);
                }
                return true;
            case HID::JButton::B:
                renderer.close();
                return true;
            case HID::JButton::Start:
                if (resetPosition) {
                    resetPosition();
                    return true;
                } else {
                    return false;
                }
            default:
                return !testing;
            }
        });
    }

    constexpr degree_t maxTurn = 45_deg;
    cv::Mat frame;
    do {
        catcher.check();

        // if (trainingDatabase) {
        //     plotter.setTitle("Training");
        // } else if (testing) {
        //     plotter.setTitle("Testing");
        // }

        if (joystick) {
            joystick->update();
        }

        renderer.update(poseGetter.getPose(), robot, trainingLine, testingLine);
        display.update();

        if (turnTimer.running()) {
            if (turnTimer.finished()) {
                robot.moveForward(forwardSpeed);
                std::cout << "(Turned for " << static_cast<millisecond_t>(turnTimer.elapsed()) << ")" << std::endl;
                turnTimer.stop();
            } else {
                continue;
            }
        }

        if (videoInput.readGreyscaleFrame(frame)) {
            const auto pose = poseGetter.template getPose<millimeter_t>();
            if (trainingDatabase) {
                trainingLine.append(pose);
                trainingDatabase->getRouteRecorder().record(pose.position(), pose.yaw(), frame);
            } else if (testing) {
                Timer<> t{ "Time to calculate: " };
                degree_t heading = std::get<0>(pm.getHeading(frame));
                if (heading < -maxTurn) {
                    heading = -maxTurn;
                } else if (heading > maxTurn) {
                    heading = maxTurn;
                }
                const auto turnTime = units::math::abs(heading) / robotTurnSpeed;
                const EggTimer::Duration turnTimeDuration = turnTime;
                if (turnTimeDuration > 1ms) { // If it's sub-millisecond, then ignore
                    robot.turnOnTheSpot(heading < 0_deg ? -turnSpeed : turnSpeed);
                    if (turnTimeDuration < 10ms) {
                        std::this_thread::sleep_for(turnTimeDuration);
                        robot.moveForward(forwardSpeed);
                    } else {
                        turnTimer.start(turnTimeDuration);
                    }
                } else {
                    robot.moveForward(forwardSpeed);
                }

                std::cout << "Heading: " << heading << " (" << turnTime << ")" << std::endl;
                testingLine.append(pose);
            }
        }

    } while (display.isOpen() && renderer.isOpen());
}
