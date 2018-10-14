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
#include "hid/joystick.h"
#include "net/client.h"
#include "os/net.h"
#include "robots/tank_netsink.h"
#include "video/display.h"
#include "video/netsource.h"

// Standard C++ includes
#include <chrono>
#include <exception>
#include <iostream>
#include <thread>

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

        // Enable networking on Windows
        OS::Net::WindowsNetworking net;

        // make connection to robot on default port
        Net::Client client(robotIP);
        client.runInBackground();

        // read video stream from network
        Video::NetSource video(client);

        // transmit motor commands over network
        Robots::TankNetSink motor(client);

        // add joystick for controlling Tank
        HID::Joystick joystick;
        motor.addJoystick(joystick); // send joystick events to motor

        // display video stream
        Video::Display display(video, { 1240, 500 });

        // poll joystick and video stream repeatedly
        do {
            if (!joystick.update() && !display.update()) {
                std::this_thread::sleep_for(50ms);
            }
        } while (display.isOpen());
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}
