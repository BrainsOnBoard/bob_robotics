// BoB robotics includes
#include "plog/Log.h"
#include "viz/sfml/joystick_keyboard.h"

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
namespace Viz {
namespace SFML {

const JoystickKeyboard::AxisKey JoystickKeyboard::AxisKeys[] = {
    { sf::Keyboard::Key::S, sf::Keyboard::Key::W, HID::JAxis::LeftStickVertical },
    { sf::Keyboard::Key::D, sf::Keyboard::Key::A, HID::JAxis::LeftStickHorizontal },
    { sf::Keyboard::Key::Down, sf::Keyboard::Key::Up, HID::JAxis::RightStickVertical },
    { sf::Keyboard::Key::Right, sf::Keyboard::Key::Left, HID::JAxis::RightStickHorizontal },
    { sf::Keyboard::Key::PageDown, sf::Keyboard::Key::PageUp, HID::JAxis::RightTrigger }
};
const JoystickKeyboard::ButtonKey JoystickKeyboard::ButtonKeys[] = {
    { sf::Keyboard::Key::Num1, HID::JButton::A },
    { sf::Keyboard::Key::Num2, HID::JButton::B },
    { sf::Keyboard::Key::Num3, HID::JButton::X },
    { sf::Keyboard::Key::Num4, HID::JButton::Y }
};

//------------------------------------------------------------------------
// BoBRobotics::Viz::SFML::JoystickKeyboard
//------------------------------------------------------------------------
JoystickKeyboard::JoystickKeyboard()
  : JoystickBase(0.0f)
{}

//------------------------------------------------------------------------
bool JoystickKeyboard::updateState()
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

std::unique_ptr<HID::JoystickBase<HID::JAxis, HID::JButton>>
JoystickKeyboard::createJoystick()
{
    try
    {
        return std::make_unique<HID::Joystick>(0.25f);
    }
    catch(std::runtime_error &ex)
    {
        LOGW << "Error opening joystick - \"" << ex.what() << "\" - using keyboard interface";
        return std::make_unique<JoystickKeyboard>();
    }
}

} // SFML
} // Viz
} // BoBRobotics
