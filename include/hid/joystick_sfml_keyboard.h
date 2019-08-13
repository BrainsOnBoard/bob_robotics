#pragma once

// BoB robotics includes
#include "joystick.h"

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <map>

namespace BoBRobotics {
namespace HID {

//------------------------------------------------------------------------
// BoBRobotics::HID::JoystickSFMLKeyboard
//------------------------------------------------------------------------
/*!
 * \brief Class for using keyboard (via SFML) as a joystick
 */
class JoystickSFMLKeyboard : public JoystickBase<JAxis, JButton>
{
public:
    JoystickSFMLKeyboard(sf::Window &window);

protected:
    //------------------------------------------------------------------------
    // JoystickBase virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() override;

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    sf::Window &m_Window;
    static const std::map<sf::Keyboard::Key, JAxis> NegativeAxisKeys, PositiveAxisKeys;
    static const std::map<sf::Keyboard::Key, JButton> ButtonKeys;
}; // JoystickLinux
} // HID
} // BoBRobotics
