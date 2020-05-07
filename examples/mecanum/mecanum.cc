// BoB robotics includes
#include "hid/joystick.h"
#include "robots/mecanum.h"

using namespace BoBRobotics;

int bobMain(int, char **)
{
    HID::Joystick joystick;
    Robots::Mecanum robot;
    robot.addJoystick(joystick);
    joystick.run();
    return EXIT_SUCCESS;
}
