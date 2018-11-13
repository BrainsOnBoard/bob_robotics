#pragma once

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/plot_agent.h"
#include "common/pose.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "navigation/perfect_memory.h"
#include "robots/robot.h"
#include "video/input.h"
#include "video/unwrapped_input.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"
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

namespace plt = matplotlibcpp;

using ObjectType = std::pair<std::vector<millimeter_t>, std::vector<millimeter_t>>;

template<typename PoseGetterType, typename LengthUnit>
class Plotter
{
public:
    Plotter(const std::vector<ObjectType> &objects, PoseGetterType &poseGetter,
            const Vector2<LengthUnit> xlim, const Vector2<LengthUnit> ylim)
      : m_Objects(objects)
      , m_PoseGetter(poseGetter)
      , m_Xlim(xlim)
      , m_Ylim(ylim)
      , m_Thread(&Plotter::run, this)
      , m_Running(true)
      , m_Finished(false)
    {}

    ~Plotter()
    {
        m_Running = false;
        if (m_Thread.joinable()) {
            m_Thread.join();
        }
    }

    void setTitle(const std::string& title)
    {
        std::lock_guard<std::mutex> guard(m_TitleMutex);
        m_Title = title;
    }

    bool isOpen() const { return m_Running; }

private:
    const std::vector<ObjectType> &m_Objects;
    PoseGetterType &m_PoseGetter;
    std::string m_Title;
    const Vector2<LengthUnit> m_Xlim, m_Ylim;
    std::thread m_Thread;
    std::mutex m_TitleMutex;
    std::atomic_bool m_Running;
    std::atomic_bool m_Finished;

    void run()
    {
        do {
            plt::figure(1);
            plt::clf();

            // Plot objects
            // plt::subplot(2, 1, 1);
            for (auto object : m_Objects) {
                std::vector<double> x, y;

                const auto mm2double = [](millimeter_t mm) { return mm.value(); };
                std::transform(object.first.cbegin(), object.first.cend(), std::back_inserter(x), mm2double);
                std::transform(object.second.cbegin(), object.second.cend(), std::back_inserter(y), mm2double);
                x.push_back(x[0]);
                y.push_back(y[0]);

                plt::plot(x, y);
            }

            // Plot position of robot
            plotAgent(m_PoseGetter, m_Xlim[0], m_Xlim[1], m_Ylim[0], m_Ylim[1]);

            // Set title
            {
                std::lock_guard<std::mutex> guard(m_TitleMutex);
                plt::title(m_Title);
            }

            plt::pause(0.1);
        } while (m_Running && plt::fignum_exists(1));
        plt::close();
        m_Running = false;
        m_Finished = true;
    }
}; // Plotter

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

template<typename PoseGetterType, typename LengthUnit>
void
runNavigation(Robots::Robot &robot,
              PoseGetterType &poseGetter,
              const float forwardSpeed,
              const float turnSpeed,
              Video::Input &videoInput,
              const LengthUnit xLower,
              const LengthUnit xUpper,
              const LengthUnit yLower,
              const LengthUnit yUpper,
              const std::vector<ObjectType> &objects = {})
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

    Plotter<PoseGetterType, LengthUnit> plotter(objects, poseGetter,
                                                { xLower, xUpper }, { yLower, yUpper });

    cv::Mat frame;
    // std::map<std::string, std::string> imshowKeywords{ { "cmap", "gray" } };
    do {
        catcher.check();

        if (trainingDatabase) {
            plotter.setTitle("Training");
        } else if (testing) {
            plotter.setTitle("Testing");
        }

        joystick.update();
        if (turnTimer.running()) {
            if (turnTimer.finished()) {
                robot.moveForward(forwardSpeed);
                turnTimer.stop();
            } else {
                continue;
            }
        }

        if (videoInput.readGreyscaleFrame(frame)) {
            // plt::subplot(2, 1, 2);
            // plt::imshow(frame, imshowKeywords);

            if (trainingDatabase) {
                const auto pose = poseGetter.template getPose<millimeter_t, degree_t>();
                trainingDatabase->getRouteRecorder().record(pose.first, pose.second[0], frame);
            } else if (testing) {
                Timer<> t{ "Time to calculate: " };
                const degree_t heading = std::get<0>(pm.getHeading(frame));
                std::cout << "Heading: " << heading << std::endl;

                turnTimer.start(heading / robotTurnSpeed);
                robot.turnOnTheSpot(heading < 0_deg ? -turnSpeed : turnSpeed);
            }
        }
    } while (!joystick.isPressed(HID::JButton::B) && plotter.isOpen());
}
