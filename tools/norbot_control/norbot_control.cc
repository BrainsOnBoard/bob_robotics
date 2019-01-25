// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "net/server.h"
#include "os/net.h"
#include "robots/norbot.h"
#include "video/netsink.h"
#include "video/opencvinput.h"
#include "video/panoramic.h"
#include "video/randominput.h"

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
    // Use Arduino robot
    Robots::Norbot tank;

    // Get panoramic camera
    std::unique_ptr<Video::Input> camera;
    try {
        camera = Video::getPanoramicCamera();
    } catch (std::runtime_error &e) {
        // Camera not found
        std::cerr << e.what() << std::endl;
    }

    // Try to get joystick
    std::unique_ptr<HID::Joystick> joystick;
    try {
        joystick = std::make_unique<HID::Joystick>();
        tank.addJoystick(*joystick);
    } catch (std::runtime_error &e) {
        // Joystick not found
        std::cerr << e.what() << std::endl;
    }

    // Listen for incoming connection on default port
    Net::Server server;
    auto connection = server.waitForConnection();

    // Read motor commands from network
    tank.readFromNetwork(connection);

    std::unique_ptr<Video::NetSink> netSink;
    if (camera) {
        // Stream camera synchronously over network
        netSink = std::make_unique<Video::NetSink>(connection, camera->getOutputSize(), camera->getCameraName());
    }

    // Run server in background,, catching any exceptions for rethrowing
    auto &catcher = BackgroundExceptionCatcher::getInstance();
    catcher.trapSignals(); // Catch Ctrl-C
    connection.runInBackground();

    cv::Mat frame;
    while (true) {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        const bool joystickUpdate = joystick && joystick->update();
        const bool cameraUpdate = camera && camera->readFrame(frame);

        // If there's a new frame, send it, else sleep
        if (cameraUpdate) {
            netSink->sendFrame(frame);
        } else if (!joystickUpdate) {
            std::this_thread::sleep_for(25ms);
        }
    }

    return EXIT_SUCCESS;
}
