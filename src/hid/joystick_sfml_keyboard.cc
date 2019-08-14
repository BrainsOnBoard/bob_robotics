// BoB robotics includes
#include "hid/joystick_sfml_keyboard.h"

template<class T, class Func>
bool runIfFound(const std::map<sf::Keyboard::Key, T> &m,
                sf::Keyboard::Key key,
                Func f)
{
    auto iter = m.find(key);
    if (iter == m.end()) { // not present
        return false;
    }

    // Run f with the value corresponding to key
    f(iter->second);
    return true;
}

namespace BoBRobotics {
namespace HID {

const std::map<sf::Keyboard::Key, JAxis> JoystickSFMLKeyboard::NegativeAxisKeys = {
    { sf::Keyboard::Key::W, JAxis::LeftStickVertical },
    { sf::Keyboard::Key::A, JAxis::LeftStickHorizontal },
    { sf::Keyboard::Key::Up, JAxis::RightStickVertical },
    { sf::Keyboard::Key::Left, JAxis::RightStickHorizontal },
    { sf::Keyboard::Key::PageUp, JAxis::RightTrigger }
};
const std::map<sf::Keyboard::Key, JAxis> JoystickSFMLKeyboard::PositiveAxisKeys = {
    { sf::Keyboard::Key::S, JAxis::LeftStickVertical },
    { sf::Keyboard::Key::D, JAxis::LeftStickHorizontal },
    { sf::Keyboard::Key::Down, JAxis::RightStickVertical },
    { sf::Keyboard::Key::Right, JAxis::RightStickHorizontal },
    { sf::Keyboard::Key::PageDown, JAxis::RightTrigger }
};
const std::map<sf::Keyboard::Key, JButton> JoystickSFMLKeyboard::ButtonKeys = {
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
    sf::Event event;
    if (!m_Window.pollEvent(event)) {
        return false;
    }
    bool down = event.type == sf::Event::KeyPressed;
    if (!down && event.type != sf::Event::KeyReleased) {
        return false;
    }
    auto key = event.key.code;

    // Check for negative axis keys
    bool didRun = runIfFound(NegativeAxisKeys, key,
        [&](JAxis axis)
        {
            this->setState(axis, down ? -1.f : 0.f, false);
        });
    if (didRun) {
        return true;
    }

    // Check for positive axis keys
    didRun = runIfFound(PositiveAxisKeys, key,
        [&](JAxis axis)
        {
            this->setState(axis, down ? 1.f : 0.f, false);
        });
    if (didRun) {
        return true;
    }

    // Check for button keys
    didRun = runIfFound(ButtonKeys, key,
        [&](JButton button)
        {
            if (down) {
                this->setPressed(button, false);
            } else {
                this->setReleased(button, false);
            }
        });

    return didRun;
}
} // HID
} // BoBRobotics
