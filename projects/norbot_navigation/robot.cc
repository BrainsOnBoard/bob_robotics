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
#include "robots/tank.h"
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
    OS::Net::WindowsNetworking::initialise();

    // Listen for incoming connection on default port
    Net::Server server;

    // Default panoramic camera
    auto camera = Video::getPanoramicCamera();

#ifdef NO_I2C_ROBOT
    Robots::Tank tank;
#else
    // Use Arduino robot
    Robots::Norbot tank;
#endif

    while (true) {
        auto connection = server.waitForConnection();

        // Stream camera synchronously over network
        Video::NetSink netSink(connection, camera->getOutputSize(), camera->getCameraName());

        // Read motor commands from network
        tank.readFromNetwork(connection);

        // Run server in background,, catching any exceptions for rethrowing
        auto &catcher = BackgroundExceptionCatcher::getInstance();
        connection.runInBackground();

        try {
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
        } catch (Net::SocketClosedError const &) {
            std::cout << "Connection closed" << std::endl;
        }
    }

    return EXIT_SUCCESS;
}