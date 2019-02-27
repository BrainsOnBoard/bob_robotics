// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "net/server.h"
#include "os/net.h"
#include "robots/norbot.h"
#include "robots/tank_netsink.h"
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
    std::unique_ptr<Video::Input> camera;
    std::unique_ptr<HID::Joystick> joystick;
    std::unique_ptr<Video::NetSink> netSink;
    std::unique_ptr<Net::Client> client;
    std::unique_ptr<Robots::Tank> tank;

    // Listen for incoming connection on default port
    Net::Server server;
    auto connection = server.waitForConnection();

    // Get panoramic camera
    try {
        camera = Video::getPanoramicCamera();
    } catch (std::runtime_error &e) {
        // Camera not found
        std::cerr << e.what() << std::endl;
    }
    if (camera) {
        // Stream camera synchronously over network
        netSink = std::make_unique<Video::NetSink>(connection, camera->getOutputSize(), camera->getCameraName());
    }

    // Try to connect to servos over I2C and if that fails, try to connect to EV3
    try {
        tank = std::make_unique<Robots::Norbot>();
    } catch (std::runtime_error &) {
        std::cout << "Trying to connect to EV3..." << std::endl;
        client = std::make_unique<Net::Client>("10.42.0.130");
        tank = std::make_unique<Robots::TankNetSink>(*client);
    }

    // Read motor commands from network
    tank->readFromNetwork(connection);

    // Try to get joystick
    try {
        joystick = std::make_unique<HID::Joystick>();
        tank->addJoystick(*joystick);
    } catch (std::runtime_error &e) {
        // Joystick not found
        std::cerr << e.what() << std::endl;
    }

    // Run server in background,, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    connection.runInBackground();
    if (client) {
        client->runInBackground();
    }

    cv::Mat frame;
    while (connection.isOpen()) {
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
