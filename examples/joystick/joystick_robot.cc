// BoB robotics includes
#include "hid/joystick.h"
#include "hid/robot_control.h"
#include "robots/robot_type.h"

#include <iostream>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;

int bobMain(int, char **)
{
    constexpr float joystickDeadzone = 0.25f;

    // Create joystick interface
    HID::Joystick joystick(joystickDeadzone);

    // Create motor interface
    ROBOT_TYPE robot;

    do {
        // Read joystick
        joystick.update();

        // Use joystick to drive motor
        HID::drive(robot, joystick);

    } while(!joystick.isDown(HID::JButton::B));

    return 0;
}
