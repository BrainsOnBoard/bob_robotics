// C++ includes
#include <stdexcept>

// local includes
#include "joystick.h"

namespace GeNNRobotics {
namespace Robots {
BebopJoystick::BebopJoystick(Bebop *bebop)
  : m_Bebop(bebop)
{
    setCallback([=](HID::Event &js) { eventCallback(js); });
}

void
BebopJoystick::onButtonEvent(HID::Event &js)
{
    if (m_ButtonCallback && m_ButtonCallback(js)) {
        return;
    }

    switch (js.button()) {
    case HID::Button::A:
        m_Bebop->takeOff();
        break;
    case HID::Button::B:
        m_Bebop->land();
        break;
    }
}

void
BebopJoystick::onAxisEvent(HID::Event &js)
{
    float f;

    switch (js.axis()) {
    case HID::Axis::RightStickHorizontal:
        f = maxbank * js.axisValue();
        m_Bebop->setRoll((i8) f);
        break;
    case HID::Axis::RightStickVertical:
        f = -maxbank * js.axisValue();
        m_Bebop->setPitch((i8) f);
        break;
    case HID::Axis::LeftStickHorizontal:
        f = maxyaw * js.axisValue();
        m_Bebop->setYaw((i8) f);
        break;
    case HID::Axis::LeftStickVertical:
        f = -maxup * js.axisValue();
        m_Bebop->setUpDown((i8) f);
        break;
    }
}

void
BebopJoystick::eventCallback(HID::Event &js)
{
    if (!js.isAxis) {
        onButtonEvent(js);
    } else {
        onAxisEvent(js);
    }
}
} // Robots
} // GeNNRobotics
