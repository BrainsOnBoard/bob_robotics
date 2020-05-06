// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/surveyor.h"

using namespace BoBRobotics;

int bobMain(int, char **)
{
    HID::Joystick joystick;
    Robots::Surveyor robot("192.168.1.1", 2000);
    robot.addJoystick(joystick);
    joystick.run();
    return EXIT_SUCCESS;
}
