// Windows headers
#include "os/windows_include.h"

// C++ includes
#include <chrono>
#include <iostream>
#include <thread>

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "hid/joystick.h"
#include "net/client.h"
#include "os/net.h"
#include "robots/tank_netsink.h"
#include "video/display.h"
#include "video/netsource.h"

using namespace BoBRobotics;
using namespace std::literals;

int
main(int argc, char **argv)
{
    try {
        std::string robotIP;
        if (argc == 2) {
            // get robot IP from commandline argument
            robotIP = argv[1];
        } else {
            // get robot IP from terminal
            std::cout << "Robot IP [127.0.0.1]: ";
            std::getline(std::cin, robotIP);
            if (robotIP.empty()) {
                robotIP = "127.0.0.1";
            }
        }

        // enable networking on Windows
        OS::Net::WindowsNetworking net;

        // make connection to robot on default port
        Net::Client client(robotIP);
        BackgroundException::enableCatching();
        client.runInBackground();

        // transmit motor commands over network
        Robots::TankNetSink robot(client);

        // add joystick for controlling Tank
        HID::Joystick joystick;
        robot.addJoystick(joystick); // send joystick events to robot

        Video::NetSource video(client);
        Video::Display display(video, { 1240, 500 });
        while (display.isOpen()) {
            bool joystickUpdate = joystick.update();
            bool displayUpdate = display.update();
            if (!joystickUpdate && !displayUpdate) {
                std::this_thread::sleep_for(25ms);
            }

            // check for exceptions on background thread
            BackgroundException::check();
        }
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}
