#include "../hid/joystick.h"
#include "../robots/norbot.h"

int main()
{
    constexpr float joystickDeadzone = 0.25f;
    
    // Create joystick interface
    GeNNRobotics::HID::Joystick joystick(joystickDeadzone);
    
    // Create motor interface
    GeNNRobotics::Robots::Norbot robot;

    do {
        // Read joystick
        joystick.update();
        
        // Use joystick to drive motor
        robot.drive(joystick);

    } while(!joystick.isDown(GeNNRobotics::HID::JButton::B));
    
    return 0;
}