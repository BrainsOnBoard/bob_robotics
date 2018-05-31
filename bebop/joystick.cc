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
    addHandler([this](HID::Event &js) { return eventCallback(js); });
}

bool
BebopJoystick::onButtonEvent(HID::Event &js)
{
    if (m_ButtonCallback && m_ButtonCallback(js)) {
        return false;
    }

    switch (js.button()) {
    case HID::Button::A:
        m_Bebop->takeOff();
        return true;
        break;
    case HID::Button::B:
        m_Bebop->land();
        return true;
        break;
    }

    return false;
}

bool
BebopJoystick::onAxisEvent(HID::Event &js)
{
    float f;

    /*
     * setRoll/Pitch etc. all take values between -100 and 100. We cap these
     * values for the joystick code to make the drone more controllable.
     */
    switch (js.axis()) {
    case HID::Axis::RightStickHorizontal:
        f = round(MaxBank * js.axisValue());
        m_Bebop->setRoll(static_cast<int8_t>(f));
        return true;
        break;
    case HID::Axis::RightStickVertical:
        f = round(-MaxBank * js.axisValue());
        m_Bebop->setPitch(static_cast<int8_t>(f));
        return true;
        break;
    case HID::Axis::LeftStickHorizontal:
        f = round(MaxYaw * js.axisValue());
        m_Bebop->setYaw(static_cast<int8_t>(f));
        return true;
        break;
    case HID::Axis::LeftStickVertical:
        f = round(-MaxUp * js.axisValue());
        m_Bebop->setUpDown(static_cast<int8_t>(f));
        return true;
        break;
    }

    return false;
}

bool
BebopJoystick::eventCallback(HID::Event &js)
{
    if (!js.isAxis) {
        return onButtonEvent(js);
    } else {
        return onAxisEvent(js);
    }
}
} // Robots
} // GeNNRobotics
