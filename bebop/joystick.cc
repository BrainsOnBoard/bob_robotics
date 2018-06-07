// C includes
#include <cmath>

// C++ includes
#include <stdexcept>

// local includes
#include "joystick.h"

namespace GeNNRobotics {
namespace Robots {
BebopJoystick::BebopJoystick(Bebop *bebop)
  : m_Bebop(bebop)
{
    addHandler([this](HID::JAxis axis, float value) { return onAxisEvent(axis, value); });
    addHandler([this](HID::JButton button, bool pressed) { return onButtonEvent(button, pressed); });
}

bool
BebopJoystick::onButtonEvent(HID::JButton button, bool pressed)
{
    // we only care about button presses
    if (!pressed) {
        return false;
    }

    // A = take off; B = land
    switch (button) {
    case HID::JButton::A:
        m_Bebop->takeOff();
        return true;
        break;
    case HID::JButton::B:
        m_Bebop->land();
        return true;
        break;
    }

    // otherwise signal that we haven't handled event
    return false;
}

bool
BebopJoystick::onAxisEvent(HID::JAxis axis, float value)
{
    float f;

    /*
     * setRoll/Pitch etc. all take values between -100 and 100. We cap these
     * values for the joystick code to make the drone more controllable.
     */
    switch (axis) {
    case HID::JAxis::RightStickHorizontal:
        f = round(MaxBank * value);
        m_Bebop->setRoll(static_cast<int8_t>(f));
        return true;
        break;
    case HID::JAxis::RightStickVertical:
        f = round(-MaxBank * value);
        m_Bebop->setPitch(static_cast<int8_t>(f));
        return true;
        break;
    case HID::JAxis::LeftStickHorizontal:
        f = round(MaxYaw * value);
        m_Bebop->setYaw(static_cast<int8_t>(f));
        return true;
        break;
    case HID::JAxis::LeftStickVertical:
        f = round(-MaxUp * value);
        m_Bebop->setUpDown(static_cast<int8_t>(f));
        return true;
        break;
    }

    // otherwise signal that we haven't handled event
    return false;
}
} // Robots
} // GeNNRobotics
