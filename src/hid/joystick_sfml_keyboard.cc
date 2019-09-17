// BoB robotics includes
#include "hid/joystick_sfml_keyboard.h"

namespace BoBRobotics {
namespace HID {

const JoystickSFMLKeyboard::AxisKey JoystickSFMLKeyboard::AxisKeys[] = {
    { sf::Keyboard::Key::S, sf::Keyboard::Key::W, JAxis::LeftStickVertical },
    { sf::Keyboard::Key::D, sf::Keyboard::Key::A, JAxis::LeftStickHorizontal },
    { sf::Keyboard::Key::Down, sf::Keyboard::Key::Up, JAxis::RightStickVertical },
    { sf::Keyboard::Key::Right, sf::Keyboard::Key::Left, JAxis::RightStickHorizontal },
    { sf::Keyboard::Key::PageDown, sf::Keyboard::Key::PageUp, JAxis::RightTrigger }
};
const JoystickSFMLKeyboard::ButtonKey JoystickSFMLKeyboard::ButtonKeys[] = {
    { sf::Keyboard::Key::Num1, JButton::A },
    { sf::Keyboard::Key::Num2, JButton::B },
    { sf::Keyboard::Key::Num3, JButton::X },
    { sf::Keyboard::Key::Num4, JButton::Y }
};

//------------------------------------------------------------------------
// BoBRobotics::HID::JoystickSFMLKeyboard
//------------------------------------------------------------------------
JoystickSFMLKeyboard::JoystickSFMLKeyboard(sf::Window &window)
  : JoystickBase(0.0f), m_Window(window)
{}

//------------------------------------------------------------------------
bool JoystickSFMLKeyboard::updateState()
{
    bool changed = false;

    // Check for keypresses related to axes
    for (auto &axisKey : AxisKeys) {
        float newVal;
        bool posDown = sf::Keyboard::isKeyPressed(axisKey.positiveKey);
        bool negDown = sf::Keyboard::isKeyPressed(axisKey.negativeKey);
        if (!posDown && !negDown) {
            newVal = 0.f;
        } else if (posDown) {
            newVal = 1.f;
        } else {
            newVal = -1.f;
        }

        // Check if the axis value will be changed
        if (newVal != getState(axisKey.axis)) {
            changed = true;
            setState(axisKey.axis, newVal, false);
        }
    }

    // Check for keypresses related to buttons
    for (auto &buttonKey : ButtonKeys) {
        bool down = sf::Keyboard::isKeyPressed(buttonKey.key);
        if (isDown(buttonKey.button) != down) {
            changed = true;
            if (down) {
                setPressed(buttonKey.button, false);
            } else {
                setReleased(buttonKey.button, false);
            }
        }
    }

    return changed;
}
} // HID
} // BoBRobotics
