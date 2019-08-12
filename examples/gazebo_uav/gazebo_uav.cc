// BoB robotics includes
#include "hid/joystick.h"
#include "robots/gazebo/camera.h"
#include "robots/gazebo/node.h"
#include "robots/gazebo/uav.h"
#include "video/display.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;

int
main(int argc, char **argv)
{
    /************************************Gazebo setup************/

    // Create our node for publishing joystick values
    gazebo::transport::NodePtr node = Gazebo::getNode();

    /************************************Gazebo setup end************/
    std::unique_ptr<Video::Display> display;
    std::unique_ptr<Gazebo::Camera> cam;
    switch(argc){
        case 1:
            std::cout << "Camera disabled.\n";
            break;
        case 3: // Initialize gazebo camera if more than 2 arguments are provided (display switch and camera url)
            std::cout << "Display switch enabled.\n";
            if(strcmp(argv[1], "-p") == 0) {
                std::cout << "Using panoramic camera.\n";
                cam = std::make_unique<Gazebo::Camera>(node, argv[2], true);
                display = std::make_unique<Video::Display>(*cam, cv::Size(640,320)); //unwrap resolution needs to be supplied
            }
            else{
                std::cerr << "Unsupported flag\n";
            }
            display->runInBackground();
            break;
        default:
            std::cout <<"Usage ./bob_iris [-s,-p] [camera_url]\n";

    }
    Robots::Gazebo::UAV iris(node); // QuadCopter agent
    HID::Joystick joystick(0.25f);
    iris.addJoystick(joystick);

    std::cout << "Iris connected and ready for liftoff!" << std::endl;

    do {
        // Check for joystick events
        if (!joystick.update()) {
            // A small delay so we don't hog CPU
            std::this_thread::sleep_for(5ms);
        }
    } while (!joystick.isPressed(HID::JButton::X));

    // Make sure to shut everything down.
    display->close();
    Gazebo::shutDown();
    std::cout <<"Shutting down...\n";
}
