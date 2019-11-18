// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/mecanum.h"

using namespace BoBRobotics;

int bob_main(int, char **)
{
    HID::Joystick joystick;
    Robots::Mecanum robot;
    robot.addJoystick(joystick);
    joystick.run();
    return EXIT_SUCCESS;
}
