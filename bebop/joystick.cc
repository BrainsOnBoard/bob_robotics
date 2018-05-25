// C++ includes
#include <stdexcept>

// local includes
#include "joystick.h"

namespace GeNNRobotics {
namespace Robots {
BebopJoystick::BebopJoystick(Bebop *bebop)
  : m_Bebop(bebop)
{
    m_Joystick.startThread(EventCallback, this);
}

void
BebopJoystick::OnButtonEvent(HID::Event *js)
{
    if (m_ButtonCallback && m_ButtonCallback(js)) {
        return;
    }
    if (!js->value) {
        return;
    }

    switch (js->button()) {
    case HID::Button::A:
        m_Bebop->takeOff();
        break;
    case HID::Button::B:
        m_Bebop->land();
        break;
    }
}

void
BebopJoystick::OnAxisEvent(HID::Event *js)
{
    float f;

    switch (js->axis()) {
    case HID::Axis::RightStickHorizontal:
        f = maxbank * (float) (js->value) /
            (float) numeric_limits<__s16>::max();
        m_Bebop->setRoll((i8) f);
        break;
    case HID::Axis::RightStickVertical:
        f = maxbank * (float) (-js->value) /
            (float) numeric_limits<__s16>::max();
        m_Bebop->setPitch((i8) f);
        break;
    case HID::Axis::LeftStickHorizontal:
        f = maxyaw * (float) (js->value) / (float) numeric_limits<__s16>::max();
        m_Bebop->setYaw((i8) f);
        break;
    case HID::Axis::LeftStickVertical:
        f = maxup * (float) (-js->value) / (float) numeric_limits<__s16>::max();
        m_Bebop->setUpDown((i8) f);
        break;
    }
}

void
BebopJoystick::EventCallback(HID::Event *js, void *data)
{
    BebopJoystick *cont = reinterpret_cast<BebopJoystick *>(data);

    if (js->isAxis) {
        cont->OnButtonEvent(js);
    } else {
        cont->OnAxisEvent(js);
    }
}
} // Robots
} // GeNNRobotics
