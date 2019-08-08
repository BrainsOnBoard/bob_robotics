/*
 * Example program for controlling a robot from a computer. The robot can be
 * controlled with a joystick and the robot's camera stream will be displayed on
 * screen. Press escape to exit.
 */

// Windows headers
#include "os/windows_include.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/logging.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "os/net.h"
#include "robots/tank_netsink.h"
#include "video/display.h"
#include "video/netsource.h"

// Standard C++ includes
#include <chrono>
#include <exception>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

int
bob_main(int argc, char **argv)
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
    Robots::TankNetSink tank(client);

    // Run client on background thread, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    // Add joystick for controlling robot
    HID::Joystick joystick;
    tank.addJoystick(joystick);

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
