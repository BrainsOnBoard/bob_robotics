// BoB robotics includes
#include "robots/mecanum.h"

#include <iostream>

using namespace BoBRobotics;


int bobMain(int, char **)
{
    
    
    // Create joystick interface
    HID::Joystick joystick(0.25f);

    // Create motor interface
    Robots::Mecanum robot;

    do {
        // Read joystick
        joystick.update();

        const bool reverse = joystick.isDown(HID::JButton::LB);
        
        // Use joystick to drive motor
        if(joystick.isDown(HID::JButton::A)) {
            std::cout << "Motor 1";
            std::cout << "(" << (reverse ? "Reverse" : "Forward") << ")" << std::endl;
            robot.driveMotors(reverse ? -1.0f : 1.0f, 0.0f, 0.0f, 0.0f);
        }
        else if(joystick.isDown(HID::JButton::B)) {
            std::cout << "Motor 2";
            std::cout << "(" << (reverse ? "Reverse" : "Forward") << ")" << std::endl;
            robot.driveMotors(0.0f, reverse ? -1.0f : 1.0f, 0.0f, 0.0f);
        }
        else if(joystick.isDown(HID::JButton::X)) {
            std::cout << "Motor 3";
            std::cout << "(" << (reverse ? "Reverse" : "Forward") << ")" << std::endl;
            robot.driveMotors(0.0f, 0.0f, reverse ? -1.0f : 1.0f, 0.0f);
        }
        else if(joystick.isDown(HID::JButton::Y)) {
            std::cout << "Motor 4";
            std::cout << "(" << (reverse ? "Reverse" : "Forward") << ")" << std::endl;
            robot.driveMotors(0.0f, 0.0f, 0.0f, reverse ? -1.0f : 1.0f);
        }
        else {
            robot.driveMotors(0.0f, 0.0f, 0.0f, 0.0f);
        }

    } while(!joystick.isDown(HID::JButton::Start));

    return 0;
    
    return EXIT_SUCCESS;
}
