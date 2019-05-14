/*
 * Example program to be run on robot, or locally on a desktop as a server.
 *
 * If you don't want to build in I2C support (e.g. if running on desktop) then
 * make the program with:
 *     NO_I2C=1 make
 *
 * Use the corresponding "computer" program to connect to the server.
 */

// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C
#endif

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "net/server.h"
#include "os/net.h"
#include "robots/tank.h"
#include "video/netsink.h"
#include "video/opencvinput.h"
#include "video/panoramic.h"
#include "video/randominput.h"

#ifndef NO_I2C
#include "robots/norbot.h"
#endif

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

void
run(Video::Input &camera)
{
    // Enable networking on Windows
    OS::Net::WindowsNetworking::initialise();

    // Listen for incoming connection on default port
    Net::Server server;
    auto connection = server.waitForConnection();

    // Stream camera synchronously over network
    Video::NetSink netSink(connection, camera.getOutputSize(), camera.getCameraName());

#ifdef NO_I2C
    // Output motor commands to terminal
    Robots::Tank tank;
#else
    // Use Arduino robot
    Robots::Norbot tank;
#endif

    // Read motor commands from network
    tank.readFromNetwork(connection);

    // Run server in background,, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    connection.runInBackground();

    // Send frames over network
    cv::Mat frame;
    while (connection.isOpen()) {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        // If there's a new frame, send it, else sleep
        if (camera.readFrame(frame)) {
            netSink.sendFrame(frame);
        } else {
            std::this_thread::sleep_for(25ms);
        }
    }
}

int
bob_main(int argc, char **argv)
{
    try {
        /*
         * Command-line argument can be:
         * - An integer (indicating number of video device)
         * - "random" for random input
         * - or a string indicating a video device
         */
        if (argc > 1) {
            try {
                // Try to parse the argument as an integer
                Video::OpenCVInput camera(std::stoi(argv[1]));
                run(camera);
            } catch (std::invalid_argument &) {
                // ...and fall back on treating it as a string
                if (strcmp(argv[1], "random") == 0) {
                    Video::RandomInput<> camera({ 500, 250 }, "webcam360");
                    run(camera);
                } else {
                    Video::OpenCVInput camera(argv[1]);
                    run(camera);
                }
            }
        } else {
            // Otherwise use the default panoramic camera
            auto camera = Video::getPanoramicCamera();
            run(*camera);
        }
    } catch (Net::SocketClosedError &) {
        // The connection was closed on purpose: do nothing
        std::cout << "Connection closed" << std::endl;
    }

    return EXIT_SUCCESS;
}
