// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/logging.h"
#include "common/main.h"
#include "net/server.h"
#include "robots/ev3/ev3.h"
#include "video/netsink.h"
#include "video/panoramic.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

int bob_main(int, char **)
{
    std::unique_ptr<Video::Input> camera;
    std::unique_ptr<Video::NetSink> netSink;

    // Listen for incoming connection on default port
    Net::Server server;
    auto connection = server.waitForConnection();

    // Read motor commands from network
    Robots::EV3 tank;
    tank.setMaximumSpeedProportion(0.7f); // Sensible default
    tank.readFromNetwork(connection);

    // Get panoramic camera
    try {
        camera = Video::getPanoramicCamera();
    } catch (std::runtime_error &e) {
        // Camera not found
        LOGW << e.what();
    }
    if (camera) {
        // Stream camera synchronously over network
        netSink = std::make_unique<Video::NetSink>(connection, camera->getOutputSize(), camera->getCameraName());
    } else {
        // Run server on main thread
        connection.run();
        return EXIT_SUCCESS;
    }

    // Run server in background,, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    connection.runInBackground();

    cv::Mat frame;
    while (connection.isOpen()) {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        // If there's a new frame, send it, else sleep
        if (camera->readFrame(frame)) {
            netSink->sendFrame(frame);
        } else {
            std::this_thread::sleep_for(25ms);
        }
    }

    return EXIT_SUCCESS;
}
