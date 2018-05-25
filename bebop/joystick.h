#pragma once

#include "bebop.h"
#include "../hid/joystick.h"

namespace GeNNRobotics {
namespace Robots {
using ButtonEvent = bool (*)(HID::Event *js);

class BebopJoystick
{
public:
    ButtonEvent m_ButtonCallback = nullptr;
    BebopJoystick(Bebop *bebop);

private:
    Bebop *m_Bebop;
    HID::Joystick m_Joystick;
    static constexpr float maxbank = 50;
    static constexpr float maxup = 50;
    static constexpr float maxyaw = 100;

    void OnButtonEvent(HID::Event *js);
    void OnAxisEvent(HID::Event *js);

    static void EventCallback(HID::Event *js, void *data);
}; // BebopJoystick
}  // Robots
}  // GeNNRobotics
