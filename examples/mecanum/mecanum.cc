// BoB robotics includes
#include "robots/omni2d/mecanum.h"

#include <iostream>

using namespace BoBRobotics;


int bobMain(int, char **)
{
    // Create joystick interface
    HID::Joystick joystick(0.25f);

    // Create motor interface
    Robots::Omni2D::Mecanum robot;

    std::cout << "Hold down joystick buttons A, B, X or Y to drive an individual motor and hold down LB to reverse." << std::endl;
    std::cout << "If wrong motor turns, or motors turn in wrong direction, swap wires going from motor shield." << std::endl;
    do {
        // Read joystick
        joystick.update();

        // Hold down
        const bool reverse = joystick.isDown(HID::JButton::LB);

        // Use joystick to drive motor
        if(joystick.isDown(HID::JButton::A)) {
            std::cout << "Motor 1 (front right)";
            std::cout << "(" << (reverse ? "Reverse" : "Forward") << ")" << std::endl;
            robot.driveMotors(reverse ? -1.0f : 1.0f, 0.0f, 0.0f, 0.0f);
        }
        else if(joystick.isDown(HID::JButton::B)) {
            std::cout << "Motor 2 (front left)";
            std::cout << "(" << (reverse ? "Reverse" : "Forward") << ")" << std::endl;
            robot.driveMotors(0.0f, reverse ? -1.0f : 1.0f, 0.0f, 0.0f);
        }
        else if(joystick.isDown(HID::JButton::X)) {
            std::cout << "Motor 3 (back right)";
            std::cout << "(" << (reverse ? "Reverse" : "Forward") << ")" << std::endl;
            robot.driveMotors(0.0f, 0.0f, reverse ? -1.0f : 1.0f, 0.0f);
        }
        else if(joystick.isDown(HID::JButton::Y)) {
            std::cout << "Motor 4 (back left)";
            std::cout << "(" << (reverse ? "Reverse" : "Forward") << ")" << std::endl;
            robot.driveMotors(0.0f, 0.0f, 0.0f, reverse ? -1.0f : 1.0f);
        }
        else {
            robot.driveMotors(0.0f, 0.0f, 0.0f, 0.0f);
        }

    } while(!joystick.isDown(HID::JButton::Start));

    return EXIT_SUCCESS;
}
