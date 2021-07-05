#pragma once

// BoB robotics includes
#include "hid/joystick.h"

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <memory>

namespace BoBRobotics {
namespace Viz {

//------------------------------------------------------------------------
// BoBRobotics::Viz::JoystickKeyboard
//------------------------------------------------------------------------
/*!
 * \brief Class for using keyboard (via SFML) as a joystick
 */
class JoystickKeyboard : public HID::JoystickBase<HID::JAxis, HID::JButton>
{
public:
    JoystickKeyboard();

    static std::unique_ptr<HID::JoystickBase<HID::JAxis, HID::JButton>> createJoystick();

protected:
    //------------------------------------------------------------------------
    // JoystickBase virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() override;

private:
    struct AxisKey {
        sf::Keyboard::Key positiveKey, negativeKey;
        HID::JAxis axis;
    };
    struct ButtonKey {
        sf::Keyboard::Key key;
        HID::JButton button;
    };

    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    static const AxisKey AxisKeys[];
    static const ButtonKey ButtonKeys[];
}; // JoystickLinux
} // HID
} // BoBRobotics
