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

#ifndef NO_I2C_ROBOT
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
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace BoBRobotics;
using namespace units::length;
using namespace std::literals;

namespace plt = matplotlibcpp;

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
    const std::string objectsPath = argc < 2 ? "../../tools/vicon_arena_constructor/objects.yaml"
                                             : argv[1];

    std::vector<std::pair<std::vector<millimeter_t>, std::vector<millimeter_t>>> objects;
    {
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
    }

    Vicon::UDPClient<> vicon(51001);

    auto cam = Video::getPanoramicCamera();
    const auto unwrapper = cam->createUnwrapper(unwrapResolution);

#ifdef NO_I2C_ROBOT
    // Output motor commands to terminal
    Robots::Tank tank;
#else
    // Use Arduino robot
    Robots::Norbot tank;
#endif

    // Control robot with joystick
    HID::Joystick joystick;
    tank.addJoystick(joystick);
    std::cout << "Joystick opened" << std::endl;

    // Make new image database for training images
    auto trainingDatabase = createTrainingDatabase();
    auto route = trainingDatabase.getRouteRecorder();
    auto &metadata = route.getMetadataWriter();
    metadata << "camera" << *cam
             << "needsUnwrapping" << false
             << "isGreyscale" << true
             << "unwrapper" << unwrapper;

    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    // Poll joystick
    bool trainingMode = false;
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
        bool cameraUpdate = trainingMode && cam->readGreyscaleFrame(frameRaw);
        if (cameraUpdate) {
            unwrapper.unwrap(frameRaw, frameUnwrapped);
            route.record(pose.getPosition<>(), pose.getAttitude()[0], frameUnwrapped);
        } else if (!joystickUpdate) {
            plt::pause(0.1);
        }

        if (joystick.isPressed(HID::JButton::Y)) {
            if (!trainingMode) {
                trainingMode = true;
                std::cout << "Recording training images" << std::endl;
            } else {
                break;
            }
        }
    }

    if (trainingMode) {
        route.save();
        std::cout << "Stopping training (" << trainingDatabase.size() << " stored)" << std::endl;
    }

    return EXIT_SUCCESS;
}
