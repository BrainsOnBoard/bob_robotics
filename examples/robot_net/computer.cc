/*
 * Example program for controlling a robot from a computer. The robot can be
 * controlled with a joystick and the robot's camera stream will be displayed on
 * screen. Press escape to exit.
 */

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "plog/Log.h"
#include "hid/joystick.h"
#include "hid/robot_control.h"
#include "net/client.h"
#include "os/net.h"
#include "robots/tank/net/sink.h"
#include "video/display.h"
#include "video/netsource.h"

// Standard C++ includes
#include <chrono>
#include <exception>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

int bobMain(int argc, char **argv)
{
    // Enable networking on Windows
    OS::Net::WindowsNetworking::initialise();

    std::string robotIP;
    if (argc == 2) {
        // Get robot IP from command-line argument
        robotIP = argv[1];
    } else {
        // Get robot IP from terminal
        LOGI << "Robot IP [127.0.0.1]: ";
        std::getline(std::cin, robotIP);
        if (robotIP.empty()) {
            robotIP = "127.0.0.1";
        }
    }

    // Make connection to robot on default port
    Net::Client client(robotIP);

    // Read video stream from network
    Video::NetSource video(client);

    // Transmit motor commands over network
    Robots::Tank::Net::Sink tank(client);

    // Run client on background thread, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    // Add joystick for controlling robot
    HID::Joystick joystick;
    HID::addJoystick(tank, joystick);

    // Display video stream on screen
    Video::Display display(video, { 1240, 500 });
    while (display.isOpen()) {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        // Poll joystick and camera for updates
        bool joystickUpdate = joystick.update();
        bool displayUpdate = display.update();
        if (!joystickUpdate && !displayUpdate) {
            std::this_thread::sleep_for(50ms);
        }
    }

    return EXIT_SUCCESS;
}
