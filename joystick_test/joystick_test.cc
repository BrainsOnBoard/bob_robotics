#include "../common/joystick.h"
#include "../robots/norbot.h"

int main()
{
    constexpr float joystickDeadzone = 0.25f;
    
    // Create joystick interface
    GeNNRobotics::Joystick joystick;
    
    // Create motor interface
    GeNNRobotics::Robots::Norbot motor;

    do {
        // Read joystick
        joystick.read();
        
        // Use joystick to drive motor
        joystick.drive(motor, joystickDeadzone);

    } while(!joystick.isButtonDown(1));
    
    return 0;
}