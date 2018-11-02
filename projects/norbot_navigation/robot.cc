/*
 * Example program to be run on robot, or locally on a desktop as a server.
 *
 * If you don't want to build in I2C support (e.g. if running on desktop) then
 * make the program with:
 *     NO_I2C_ROBOT=1 make
 *
 * Use the corresponding "computer" program to connect to the server.
 */

// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C_ROBOT
#endif

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "net/server.h"
#include "os/net.h"
#include "robots/norbot.h"
#include "video/netsink.h"
#include "video/panoramic.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

int
bob_main(int, char **)
{
    // Enable networking on Windows
    OS::Net::WindowsNetworking net;

    // Listen for incoming connection on default port
    Net::Server server;

    // Default panoramic camera
    auto camera = Video::getPanoramicCamera();

    // Stream camera synchronously over network
    Video::NetSink netSink(server, camera->getOutputSize(), camera->getCameraName());

    // Use Arduino robot
    Robots::Norbot tank;

    // Read motor commands from network
    tank.readFromNetwork(server);

    // Run server in background,, catching any exceptions for rethrowing
    auto &catcher = BackgroundExceptionCatcher::getInstance();
    catcher.trapSignals(); // Catch Ctrl-C
    server.runInBackground();

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

    return EXIT_SUCCESS;
}