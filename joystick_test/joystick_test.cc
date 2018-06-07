#include "../hid/joystick.h"
#include "../robots/motor_i2c.h"

int main()
{
    constexpr float joystickDeadzone = 0.25f;
    
    // Create joystick interface
    GeNNRobotics::HID::Joystick joystick(joystickDeadzone);
    
    // Create motor interface
    GeNNRobotics::Robots::MotorI2C motor;

    do {
        // Read joystick
        joystick.update();
        
        // Use joystick to drive motor
        motor.drive(joystick);

    } while(!joystick.isDown(GeNNRobotics::HID::JButton::B));
    
    return 0;
}