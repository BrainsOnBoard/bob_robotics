// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C_ROBOT
#endif

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "net/server.h"
#include "os/net.h"
#include "robots/tank.h"
#include "video/netsink.h"
#include "video/opencvinput.h"
#include "video/panoramic.h"
#include "video/randominput.h"

#ifndef NO_I2C_ROBOT
#include "robots/norbot.h"
#endif

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

int
bob_main(int, char **)
{
    // Enable networking on Windows
    OS::Net::WindowsNetworking::initialise();

    // Get panoramic camera
    std::unique_ptr<Video::Input> camera;
    try {
        camera = Video::getPanoramicCamera();
    } catch (std::runtime_error &e) {
        // Camera not found
        std::cerr << e.what() << std::endl;
    }

    // Listen for incoming connection on default port
    Net::Server server;
    auto connection = server.waitForConnection();

#ifdef NO_I2C_ROBOT
    // Output motor commands to terminal
    Robots::Tank tank;
#else
    // Use Arduino robot
    Robots::Norbot tank;
#endif

    // Read motor commands from network
    tank.readFromNetwork(connection);

    // Try to get joystick
    std::unique_ptr<HID::Joystick> joystick;
    try {
        joystick = std::make_unique<HID::Joystick>();
        joystick->runInBackground();
    } catch (std::runtime_error &e) {
        // Joystick not found
        std::cerr << e.what() << std::endl;
    }

    if (camera) {
        // Stream camera synchronously over network
        Video::NetSink netSink(connection, camera->getOutputSize(), camera->getCameraName());

        // Run server in background,, catching any exceptions for rethrowing
        auto &catcher = BackgroundExceptionCatcher::getInstance();
        catcher.trapSignals(); // Catch Ctrl-C
        connection.runInBackground();

        // Send frames over network
        cv::Mat frame;
        while (true) {
            // Rethrow any exceptions caught on background thread
            catcher.check();

            // If there's a new frame, send it, else sleep
            if (camera->readFrame(frame)) {
                netSink.sendFrame(frame);
            } else {
                std::this_thread::sleep_for(25ms);
            }
        }
    } else {
        connection.run();
    }

    return EXIT_SUCCESS;
}
