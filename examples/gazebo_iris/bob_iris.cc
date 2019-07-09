// BoB robotics includes
#include "common/main.h"
#include "common/gazebo_node.h"
#include "common/logging.h"
#include "hid/joystick.h"
#include "robots/gazebo_quadcopter.h"
// #include "robots/gazebo_tank.h"
// #include "video/gazebocamerainput.h"
// #include "video/display.h"

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
// using namespace BoBRobotics::Video;

int
bob_main(int argc, char **argv)
{
    /************************************Gazebo setup************/

    // Create our node for publishing joystick values
    gazebo::transport::NodePtr node = getGazeboNode();

    /************************************Gazebo setup end************/

    Robots::GazeboQuadCopter iris(node); // QuadCopter agent
    HID::Joystick joystick(0.25f);
    iris.addJoystick(joystick);

    std::cout << "Sky is the limit!" << std::endl;

    do {
        // Check for joystick events
        if (!joystick.update()) {
            // A small delay so we don't hog CPU
            std::this_thread::sleep_for(5ms);
        }
    } while (!joystick.isPressed(HID::JButton::X));
    // Make sure to shut everything down.
    // display->close();
    shutdownGazeboNode();
    std::cout <<"Shutting down...\n";

    return EXIT_SUCCESS;
}
