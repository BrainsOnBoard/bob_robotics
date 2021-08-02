// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "net/server.h"
#include "os/net.h"
#include "robots/robot_type.h"
#include "robots/tank/net/sink.h"
#include "robots/tank/net/source.h"
#include "video/netsink.h"
#include "video/opencvinput.h"
#include "video/panoramic.h"
#include "video/randominput.h"

#ifdef ROBOT_TYPE_EV3_EV3
#include "robots/ev3/mindstorms_imu.h"
#endif

// Third-party includes
#include "plog/Log.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
#include <memory>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

int bobMain(int, char **)
{
    std::unique_ptr<Video::Input> camera;
    std::unique_ptr<HID::Joystick> joystick;
    std::unique_ptr<Video::NetSink> netSink;

    // Listen for incoming connection on default port
    Net::Server server;
    auto connection = server.waitForConnection();

    // Get panoramic camera
    try {
        camera = Video::getPanoramicCamera();
    } catch (std::runtime_error &e) {
        // Camera not found
        LOGW << e.what();
    }

    // Construct tank of desired type
    Robots::ROBOT_TYPE tank;

    // Read motor commands from network
    const auto netSource = Robots::Tank::Net::createSource(*connection, tank);

    // Try to get joystick
    try {
        joystick = std::make_unique<HID::Joystick>();
        tank.addJoystick(*joystick);
    } catch (std::runtime_error &e) {
        // Joystick not found
        LOGW << e.what();
    }

#ifdef ROBOT_TYPE_EV3
    tank.setMaximumSpeedProportion(0.7f); // Sensible default

    // If an IMU is present, stream over network
    std::unique_ptr<MindstormsIMU> imu;
    try {
        imu = std::make_unique<MindstormsIMU>();
    } catch (...) {
        LOGW << "IMU not found";
    }
    if (imu) {
        LOGI << "Found Mindstorms IMU";
        imu->streamOverNetwork(connection);
    }
#endif

    if (!joystick && !camera) {
        // Run on main thread
        connection->run();
        return EXIT_SUCCESS;
    }
    if (camera) {
        // Stream camera synchronously over network
        netSink = std::make_unique<Video::NetSink>(*connection, camera->getOutputSize(), camera->getCameraName());
    }

    // Run server in background,, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    connection->runInBackground();

    cv::Mat frame;
    while (connection->isOpen()) {
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
