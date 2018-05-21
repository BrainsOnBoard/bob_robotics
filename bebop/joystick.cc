#include "joystick.h"
#include <stdexcept>

namespace GeNNRobotics {
namespace Robots {
BebopJoystick::BebopJoystick(Bebop *bebop)
  : m_Bebop(bebop)
{
    if (!m_Joystick.open()) {
        throw std::runtime_error("Could not find joystick");
    }
    m_Joystick.startThread(EventCallback, this);
}

void
BebopJoystick::OnButtonEvent(Joystick::Event *js)
{
    if (m_ButtonCallback && m_ButtonCallback(js)) {
        return;
    }
    if (!js->value) {
        return;
    }

    switch (js->number) {
    case Joystick::A:
        m_Bebop->takeOff();
        break;
    case Joystick::B:
        m_Bebop->land();
        break;
    }
}

void
BebopJoystick::OnAxisEvent(Joystick::Event *js)
{
    float f;

    switch (js->number) {
    case Joystick::RightStickHorizontal:
        f = maxbank * (float) (js->value) /
            (float) numeric_limits<__s16>::max();
        m_Bebop->setRoll((i8) f);
        break;
    case Joystick::RightStickVertical:
        f = maxbank * (float) (-js->value) /
            (float) numeric_limits<__s16>::max();
        m_Bebop->setPitch((i8) f);
        break;
    case Joystick::LeftStickHorizontal:
        f = maxyaw * (float) (js->value) / (float) numeric_limits<__s16>::max();
        m_Bebop->setYaw((i8) f);
        break;
    case Joystick::LeftStickVertical:
        f = maxup * (float) (-js->value) / (float) numeric_limits<__s16>::max();
        m_Bebop->setUpDown((i8) f);
        break;
    }
}

void
BebopJoystick::EventCallback(Joystick::Event *js, void *data)
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
