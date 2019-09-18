// BoB robotics includes
#include "antworld/tank.h"
#include "hid/joystick_sfml_keyboard.h"
#include "video/display.h"

using namespace BoBRobotics;

int main()
{
    AntWorld::Tank tank;
    auto camera = tank.getCamera();
    auto joystick = HID::JoystickSFMLKeyboard::createJoystick(tank.getWindow());
    tank.controlWithThumbsticks(*joystick);
    Video::Display display{ *camera };
    do {
        joystick->update();
        display.update();
    } while(!joystick->isPressed(HID::JButton::B) && display.isOpen());
}
