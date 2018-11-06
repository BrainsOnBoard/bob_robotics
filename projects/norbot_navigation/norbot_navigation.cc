// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C_ROBOT
#endif

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "common/plot_agent.h"
#include "common/pose.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "navigation/perfect_memory.h"
#include "robots/tank.h"
#include "vicon/udp.h"
#include "video/panoramic.h"

#ifdef NO_I2C_ROBOT
#include "net/client.h"
#include "robots/tank_netsink.h"
#include "video/netsource.h"
#else
#include "robots/norbot.h"
#endif

// Third-party includes
#include "third_party/matplotlibcpp.h"
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
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
    TrainingDatabase(const int routeNum, const Video::Input &cam, const ImgProc::OpenCVUnwrap360 &unwrapper)
      : Navigation::ImageDatabase(getRoutePath(routeNum))
      , m_Recorder(*this)
    {
        auto &metadata = m_Recorder.getMetadataWriter();
        metadata << "camera" << cam
                 << "needsUnwrapping" << false
                 << "isGreyscale" << true
                 << "unwrapper" << unwrapper;
    }

    ~TrainingDatabase()
    {
        m_Recorder.save();
        std::cout << "Stopping training (" << size() << " stored)" << std::endl;
    }

    Navigation::ImageDatabase::RouteRecorder &getRouteRecorder() { return m_Recorder; }

private:
    Navigation::ImageDatabase::RouteRecorder m_Recorder;
};

auto
loadObjects(const std::string &objectsPath)
{
    std::vector<std::pair<std::vector<millimeter_t>, std::vector<millimeter_t>>> objects;

    std::cout << "Loading object positions from " << objectsPath << "..." << std::endl;
    cv::FileStorage fs(objectsPath, cv::FileStorage::READ);
    std::vector<double> vertex(2);
    for (auto objectNode : fs["objects"]) {
        objects.emplace_back();
        for (auto vertexNode : objectNode) {
            vertexNode >> vertex;
            objects.back().first.emplace_back(vertex[0]);
            objects.back().second.emplace_back(vertex[1]);
        }
    }

    return objects;
}

int
bob_main(int argc, char **argv)
{
    const float forwardSpeed = 1.f;

    const cv::Size unwrapResolution{ 360, 75 };
    const auto objects = loadObjects("../../tools/vicon_arena_constructor/objects.yaml");

    const filesystem::path routeBasePath = "routes";
    filesystem::create_directory(routeBasePath);

    const auto robotTurnSpeed = 13.333_deg_per_s;

    // Count number of routes
    int numRoutes = 0;
    while(getRoutePath(++numRoutes).exists())
        ;
    std::unique_ptr<TrainingDatabase> trainingDatabase;

    Navigation::PerfectMemoryRotater<> pm(unwrapResolution);

    Vicon::UDPClient<> vicon(51001);

#ifdef NO_I2C_ROBOT
    std::unique_ptr<Net::Client> client;
    std::unique_ptr<Robots::Tank> tank;
    std::unique_ptr<Video::Input> cam;
    if ((argc > 1 && strcmp(argv[1], "--robot") == 0)) {
        cam = Video::getPanoramicCamera();
        tank = std::make_unique<Robots::Tank>();
    } else {
        std::string robotIP;
        if (argc == 2) {
            // Get robot IP from command-line argument
            robotIP = argv[1];
        } else {
            // Get robot IP from terminal
            std::cout << "Robot IP [127.0.0.1]: ";
            std::getline(std::cin, robotIP);
            if (robotIP.empty()) {
                robotIP = "127.0.0.1";
            }
        }

        // Make connection to robot on default port
        client = std::make_unique<Net::Client>(robotIP);

        // Output motor commands to network
        tank = std::make_unique<Robots::TankNetSink>(*client);

        // Read video stream from network
        cam = std::make_unique<Video::NetSource>(*client);

        client->runInBackground();
    }
#else
    // Use Arduino robot
    auto tank = std::make_unique<Robots::Norbot>();
    auto cam = Video::getPanoramicCamera();
#endif
    const auto unwrapper = cam->createUnwrapper(unwrapResolution);

    auto &catcher = BackgroundExceptionCatcher::getInstance();
    catcher.trapSignals();

    // Control robot with joystick
    HID::Joystick joystick;
    tank->addJoystick(joystick);
    std::cout << "Joystick opened" << std::endl;

    bool testing = false;
    bool isTurning = false;
    using TimeType = std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds>;
    TimeType turnStartTime;
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
                trainingDatabase = std::make_unique<TrainingDatabase>(numRoutes++, *cam, unwrapper);
                std::cout << "Recording training images" << std::endl;
            }
            return true;
        case HID::JButton::X:
            if (testing) {
                tank->stopMoving();
                testing = false;
                isTurning = false;
                std::cout << "Stopping testing" << std::endl;
            } else {
                if (trainingDatabase) {
                    trainingDatabase.reset();
                }

                std::cout << "Starting testing" << std::endl;
                if (pm.getNumSnapshots() == 0) {
                    const int imageStep = 1;
                    const Navigation::ImageDatabase database(getRoutePath(numRoutes - 1));
                    std::cout << "Loading " << database.size() / imageStep << " images" << std::endl;
                    pm.trainRoute(database, imageStep);
                }
                testing = true;

                tank->moveForward(forwardSpeed);
            }
            return true;
        default:
            return !testing;
        }
    });

    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
        catcher.check();
    }

    // Poll joystick
    cv::Mat frameRaw, frameUnwrapped;
    std::chrono::nanoseconds turnDuration(0);
    const auto getNow = []() { return std::chrono::high_resolution_clock::now(); };
    std::map<std::string, std::string> imshowKeywords;
    imshowKeywords["cmap"] = "gray";
    do {
        catcher.check();

        const auto pose = vicon.getObjectData(0);

        plt::figure(1);
        plt::clf();

        // Plot objects
        plt::subplot(2, 1, 1);
        for (auto object : objects) {
            std::vector<double> x, y;

            const auto mm2double = [](millimeter_t mm) { return mm.value(); };
            std::transform(object.first.cbegin(), object.first.cend(), std::back_inserter(x), mm2double);
            std::transform(object.second.cbegin(), object.second.cend(), std::back_inserter(y), mm2double);
            x.push_back(x[0]);
            y.push_back(y[0]);

            plt::plot(x, y);
            plt::xlabel("x (mm)");
            plt::ylabel("y (mm)");
        }

        // Plot position of robot
        plotAgent(pose, { -3000, 3000 }, { -3000, 3000 });
        if (trainingDatabase) {
            plt::title("Training");
        } else if (testing) {
            plt::title("Testing");
        }

        bool joystickUpdate = joystick.update();
        if (isTurning) {
            if ((turnStartTime - getNow()) >= turnDuration) {
                tank->moveForward(forwardSpeed);
                isTurning = false;
            } else {
                continue;
            }
        }

        if (cam->readGreyscaleFrame(frameRaw)) {
            unwrapper.unwrap(frameRaw, frameUnwrapped);
            plt::subplot(2, 1, 2);
            plt::imshow(frameUnwrapped, imshowKeywords);
            plt::pause(0.1);

            if (trainingDatabase) {
                trainingDatabase->getRouteRecorder().record(pose.getPosition<>(), pose.getAttitude()[0], frameUnwrapped);
            } else if (testing) {
                Timer<> t{ "Time to calculate: " };
                const degree_t heading = std::get<0>(pm.getHeading(frameUnwrapped));
                std::cout << "Heading: " << heading << std::endl;
                const nanosecond_t turnDurationUnits = heading / robotTurnSpeed;
                turnDuration = std::chrono::nanoseconds(static_cast<int64_t>(turnDurationUnits.value()));
                isTurning = true;
                turnStartTime = getNow();
                tank->turnOnTheSpot(heading < 0_deg ? 1.f : -1.f);
            }
        } else if (!joystickUpdate) {
            plt::pause(0.1);
        }
    } while (!joystick.isPressed(HID::JButton::B) && plt::fignum_exists(1));

    plt::close();

    return EXIT_SUCCESS;
}
