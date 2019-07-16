// BoB robotics includes
#include "common/logging.h"
#include "gazebo/node.h"
#include "hid/joystick.h"
#include "robots/gazebo/omni2d.h"

#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::literals;
using namespace std::literals;

int main()
{
    auto node = Gazebo::getNode();
    Robots::Gazebo::Omni2D robot(*node, 1_mps);
    HID::Joystick joystick(0.25f);
    robot.addJoystick(joystick);

    std::cout << "Drive the car using the two thumbsticks: each stick is for one motor" << std::endl;

    do {
        // Check for joystick events
        if (!joystick.update()) {
            // A small delay so we don't hog CPU
            std::this_thread::sleep_for(5ms);
        }
    } while (!joystick.isPressed(HID::JButton::B));

    // Make sure to shut everything down.
    Gazebo::shutDown();
    std::cout <<"Shutting down...\n";

    return EXIT_SUCCESS;
}
