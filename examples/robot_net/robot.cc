/*
 * Example program to be run on robot, or locally on a desktop as a server.
 *
 * Use the corresponding "computer" program to connect to the server.
 */

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "net/server.h"
#include "os/net.h"
#include "robots/robot_type.h"
#include "robots/tank/net/source.h"
#include "video/netsink.h"
#include "video/opencvinput.h"
#include "video/panoramic.h"
#include "video/randominput.h"

// Third-party includes
#include "plog/Log.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
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
    Video::NetSink netSink(*connection, camera.getOutputSize(), camera.getCameraName());

    // Initialise robot
    ROBOT_TYPE tank;

    // Read motor commands from network
    const auto netSource = Robots::Tank::Net::createSource(*connection, tank);

    // Run server in background,, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    connection->runInBackground();

    // Send frames over network
    cv::Mat frame;
    while (connection->isOpen()) {
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

int bobMain(int argc, char **argv)
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
                    Video::RandomInput<> camera({ 500, 250 });
                    camera.setCameraName("webcam360");
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
        LOGI << "Connection closed";
    }

    return EXIT_SUCCESS;
}
