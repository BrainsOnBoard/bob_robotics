// BoB robotics includes
#include "common/main.h"
#include "common/gazebo_node.h"
#include "common/logging.h"
#include "hid/joystick.h"
#include "robots/simulated_tank.h"
#include "robots/gazebo_tank.h"
#include "video/gazebocamerainput.h"
#include "video/display.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>
#include <cstring>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace BoBRobotics::Video;

int
bob_main(int argc, char **argv)
{
    /************************************Gazebo setup************/

    // Create our node for publishing joystick values
    gazebo::transport::NodePtr node = getGazeboNode();

    /************************************Gazebo setup end************/
    char* camera_url="";
    if(argc >= 3) {
        camera_url = argv[2];
    }    
    GazeboCameraInput cam(node, camera_url);
    Display display(cam);
    //Initialize Gazebo camera display if -d arguement supplied
    if(argc >= 3 && strcmp(argv[1], "-d") == 0) {
        std::cout << "Display switch enabled.\n";
        display.runInBackground();
    }

    Robots::GazeboTank robot(5_rad_per_s, node); // Tank agent
    HID::Joystick joystick(0.25f);
    robot.controlWithThumbsticks(joystick);

    std::cout << "Drive the car using the two thumbsticks: each stick is for one motor" << std::endl;

    do {
        // Check for joystick events
        if (!joystick.update()) {
            // A small delay so we don't hog CPU
            std::this_thread::sleep_for(5ms);
        }
    } while (!joystick.isPressed(HID::JButton::B));
    // Make sure to shut everything down.
    shutdownGazeboNode();
    std::cout <<"Shutting down...\n";

    return EXIT_SUCCESS;
}
