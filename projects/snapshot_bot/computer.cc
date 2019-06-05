/*
 * Example program for controlling a robot from a computer. The robot can be
 * controlled with a joystick and the robot's camera stream will be displayed on
 * screen. Press escape to exit.
 */

// Windows headers
#include "os/windows_include.h"

// C++ includes
#include <chrono>
#include <iostream>
#include <thread>

// GeNN robotics includes
#include "hid/joystick.h"
#include "net/client.h"
#include "os/net.h"
#include "robots/tank_netsink.h"
#include "video/display.h"
#include "video/netsource.h"

using namespace BoBRobotics;

int main(int argc, char **argv)
{
    std::string robotIP;
    if (argc == 2) {
        // get robot IP from commandline argument
        robotIP = argv[1];
    } else {
        // get robot IP from terminal
        LOGI << "Robot IP [127.0.0.1]: ";
        std::getline(std::cin, robotIP);
        if (robotIP.empty()) {
            robotIP = "127.0.0.1";
        }
    }

    // start networking API on Windows
    OS::Net::WindowsNetworking::initialise();

    // use a separate scope so that socket is closed before WSACleanup is called
    {
        // make connection to robot on default port
        Net::Client client(robotIP);
        client.runInBackground();

        // read video stream from network
        Video::NetSource video(client);

        const cv::Size videoSize = video.getOutputSize();
        
        cv::namedWindow("Output", cv::WINDOW_NORMAL);
        cv::resizeWindow("Output", videoSize.width * 5, videoSize.height * 5);
        
        // poll joystick and video stream repeatedly
        cv::Mat frame(videoSize, CV_8UC1);
        while(true) {
            // Read video frame
            video.readFrame(frame);
            
            cv::imshow("Output", frame);
            
            if(cv::waitKey(33) == 27) {
                break;
            }
        }
    }
}
