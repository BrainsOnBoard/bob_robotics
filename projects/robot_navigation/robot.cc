// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C_ROBOT
#endif

// BoB robotics includes
#include "hid/joystick.h"
#include "navigation/image_database.h"
#include "robots/tank.h"
#include "vicon/udp.h"
#include "video/panoramic.h"

#ifndef NO_I2C_ROBOT
#include "robots/norbot.h"
#endif

// Third-party includes
#include "third_party/path.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

Navigation::ImageDatabase createTrainingDatabase()
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
main()
{
    try {
        Vicon::UDPClient<> vicon(51001);

        auto cam = Video::getPanoramicCamera();

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
        auto trainImages = createTrainingDatabase();
        auto route = trainImages.getRouteRecorder();

        while (vicon.getNumObjects() == 0) {
            std::this_thread::sleep_for(1s);
            std::cout << "Waiting for object" << std::endl;
        }

        // Poll joystick
        bool trainingMode = false;
        cv::Mat frame;
        while (!joystick.isPressed(HID::JButton::B)) {
            bool joystickUpdate = joystick.update();
            bool cameraUpdate = trainingMode && cam->readFrame(frame);
            if (cameraUpdate) {
                auto pose = vicon.getObjectData(0);
                route.record(pose.getPosition<>(), pose.getAttitude()[0], frame);
            } else if (!joystickUpdate) {
                std::this_thread::sleep_for(10ms);
            }

            if (joystick.isPressed(HID::JButton::Y)) {
                trainingMode = !trainingMode;
                if (trainingMode) {
                    std::cout << "Entering training mode" << std::endl;
                } else {
                    route.save();
                    std::cout << "Stopping training (" << trainImages.size() << " stored)" << std::endl;
                }
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}
