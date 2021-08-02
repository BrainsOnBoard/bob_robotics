// BoB robotics includes
#include "hid/joystick.h"
#include "robots/gazebo/camera.h"
#include "robots/gazebo/node.h"
#include "robots/gazebo/tank.h"
#include "robots/tank/simulated_tank.h"
#include "video/display.h"

// Third-party includes
#include "plog/Log.h"
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
bobMain(int argc, char **argv)
{
    /************************************Gazebo setup************/

    // Create our node for publishing joystick values
    gazebo::transport::NodePtr node = Gazebo::getNode();

    /************************************Gazebo setup end************/
    std::unique_ptr<Video::Display> display;
    std::unique_ptr<Gazebo::Camera> cam;
    if (argc >= 3) { // Initialize gazebo camera if more than 2 arguments are provided (display switch and camera url)
        std::cout << "Display switch enabled.\n";
        if (strcmp(argv[1], "-p") == 0) {
            std::cout << "Using panoramic camera.\n";
            cam = std::make_unique<Gazebo::Camera>(node, argv[2], true);
            display = std::make_unique<Video::Display>(*cam, cv::Size(640, 320)); //unwrap resolution needs to be supplied
        } else if (strcmp(argv[1], "-s") == 0) {
            std::cout << "Using simple camera.\n";
            cam = std::make_unique<Gazebo::Camera>(node, argv[2], false);
            display = std::make_unique<Video::Display>(*cam); //unwrap resolution needs to be supplied
        }
    }

    Gazebo::Tank robot(1_mps, node); // Tank agent
    std::unique_ptr<HID::Joystick> joystick;
    try {
        joystick = std::make_unique<HID::Joystick>(0.25f);
        robot.controlWithThumbsticks(*joystick);
        std::cout << "Drive the car using the two thumbsticks: each stick is for one motor" << std::endl;
    } catch (std::runtime_error &) {
        LOGW << "Could not find joystick";
    }

    if (!display && !joystick) {
        std::cout << "Press any key to exit\n";
        std::cin.ignore();
    } else {
        do {
            // Check for joystick events
            bool joyUpdate = joystick && joystick->update();
            bool dispUpdate = display && display->update();
            if (!joyUpdate && !dispUpdate) {
                // A small delay so we don't hog CPU
                std::this_thread::sleep_for(5ms);
            }
        } while ((!joystick || !joystick->isPressed(HID::JButton::B)) &&
                    (!display || display->isOpen()));
    }

    // Make sure to shut everything down.
    Gazebo::shutDown();
    std::cout << "Shutting down...\n";

    return EXIT_SUCCESS;
}
