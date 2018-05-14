#pragma once

#include "bebop.h"
#include "../joystick/joystick.h"

namespace Parrot {
using ButtonEvent = bool (*)(Joystick::Event *js);

class BebopJoystick
{
public:
    ButtonEvent m_ButtonCallback = nullptr;
    BebopJoystick(Bebop *bebop);

private:
    Parrot::Bebop *m_Bebop;
    Joystick::Joystick m_Joystick;
    static constexpr float maxbank = 50;
    static constexpr float maxup = 50;
    static constexpr float maxyaw = 100;

    void OnButtonEvent(Joystick::Event *js);
    void OnAxisEvent(Joystick::Event *js);

    static void EventCallback(Joystick::Event *js, void *data);
};
}
