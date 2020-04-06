#include "../hid/joystick.h"
#include "../robots/mecanum.h"

int main()
{
    constexpr float joystickDeadzone = 0.25f;
    
    // Create joystick interface
    BoBRobotics::HID::Joystick joystick(joystickDeadzone);
    
    // Create motor interface
    BoBRobotics::Robots::Mecanum robot;

    do {
        // Read joystick
        joystick.update();
        
        // Use joystick to drive motor
        robot.drive(joystick);

    } while(!joystick.isDown(BoBRobotics::HID::JButton::B));
    
    return 0;
}
