// BoB robotics includes
#include "hid/joystick.h"
#include "robots/norbot.h"

int main()
{
    constexpr float joystickDeadzone = 0.25f;

    // Create joystick interface
    BoBRobotics::HID::Joystick joystick(joystickDeadzone);

    // Create motor interface
    BoBRobotics::Robots::Norbot robot;

    do {
        // Read joystick
        joystick.update();

        // Use joystick to drive motor
        robot.drive(joystick);

    } while(!joystick.isDown(BoBRobotics::HID::JButton::B));

    return 0;
}
