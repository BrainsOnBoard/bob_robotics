#pragma once

// BoB robotics includes
#include "joystick.h"

// SFML
#include <SFML/Graphics.hpp>

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
    struct AxisKey {
        sf::Keyboard::Key positiveKey, negativeKey;
        JAxis axis;
    };
    struct ButtonKey {
        sf::Keyboard::Key key;
        JButton button;
    };

    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    sf::Window &m_Window;
    static const AxisKey AxisKeys[];
    static const ButtonKey ButtonKeys[];
}; // JoystickLinux
} // HID
} // BoBRobotics
