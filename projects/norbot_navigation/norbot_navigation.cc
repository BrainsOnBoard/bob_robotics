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
#include "hid/joystick.h"
#include "navigation/image_database.h"
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
using namespace units::length;
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
      , m_Recorder(getRouteRecorder())
    {
        auto &metadata = m_Recorder.getMetadataWriter();
        metadata << "camera" << cam
                 << "needsUnwrapping" << false
                 << "isGreyscale" << true
                 << "unwrapper" << unwrapper;
    }

    ~TrainingDatabase()
    {
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

Navigation::ImageDatabase
createTrainingDatabase()
{
    const filesystem::path basePath = "routes";
    filesystem::create_directory(basePath);

    filesystem::path databasePath;
    int i = 1;
    do {
        databasePath = basePath / ("route" + std::to_string(i++));
    } while (databasePath.exists());

    std::cout << "Creating folder " << databasePath << std::endl;
    return databasePath;
}

int
bob_main(int argc, char **argv)
{
    const cv::Size unwrapResolution{ 360, 75 };
    const auto objects = loadObjects("../../tools/vicon_arena_constructor/objects.yaml");
    const filesystem::path routeBasePath = "routes";
    filesystem::create_directory(routeBasePath);

    // Count number of routes
    int numRoutes = 0;
    while(getRoutePath(++numRoutes).exists())
        ;
    std::unique_ptr<TrainingDatabase> trainingDatabase;

    Vicon::UDPClient<> vicon(51001);

#ifdef NO_I2C_ROBOT
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
    Net::Client client(robotIP);

    // Output motor commands to network
    Robots::TankNetSink tank(client);

    // Read video stream from network
    Video::NetSource video(client);
    Video::Input *cam = &video;
#else
    // Use Arduino robot
    Robots::Norbot tank;

    auto cam = Video::getPanoramicCamera();
#endif
    const auto unwrapper = cam->createUnwrapper(unwrapResolution);

    // Control robot with joystick
    HID::Joystick joystick;
    tank.addJoystick(joystick);
    std::cout << "Joystick opened" << std::endl;

    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    // Poll joystick
    cv::Mat frameRaw, frameUnwrapped;
    while (!joystick.isPressed(HID::JButton::B)) {
        const auto pose = vicon.getObjectData(0);

        plt::figure(1);
        plt::clf();

        // Plot objects
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
        plotAgent(pose, { -1500, 1500 }, { -1500, 1500 });

        bool joystickUpdate = joystick.update();
        bool cameraUpdate = trainingDatabase && cam->readGreyscaleFrame(frameRaw);
        if (cameraUpdate) {
            unwrapper.unwrap(frameRaw, frameUnwrapped);
            trainingDatabase->getRouteRecorder().record(pose.getPosition<>(), pose.getAttitude()[0], frameUnwrapped);
        } else if (!joystickUpdate) {
            plt::pause(0.1);
        }

        if (joystick.isPressed(HID::JButton::Y)) {
            if (trainingDatabase) {
                trainingDatabase.reset();
            } else {
                trainingDatabase = std::make_unique<TrainingDatabase>(numRoutes++, *cam, unwrapper);
                std::cout << "Recording training images" << std::endl;
            }
        }
    }

    return EXIT_SUCCESS;
}
