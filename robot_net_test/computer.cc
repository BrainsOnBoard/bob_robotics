/*
 * Example program for controlling a robot from a computer. The robot can be
 * controlled with a joystick and the robot's camera stream will be displayed on
 * screen. Press escape to exit.
 */

// Windows headers
#include "os/windows_include.h"

// C++ includes
#include <iostream>

// GeNN robotics includes
#include "net/client.h"
#include "robots/motor_joystick.h"
#include "robots/motor_netsink.h"
#include "video/display.h"
#include "video/netsource.h"

using namespace GeNNRobotics;

int
main(int argc, char **argv)
{
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

    // make connection to robot on default port
    Net::Client client(robotIP);
    client.runInBackground();

    // read video stream from network
    Video::NetSource video(client);

    // transmit motor commands over network
    Robots::MotorNetSink motor(client);

    // add joystick for controlling Motor
    Robots::MotorJoystick joystick(motor);
    joystick.runInBackground();

    // display video stream
    Video::Display display(video, cv::Size(1240, 500));
    display.run(); // run until terminated
}
