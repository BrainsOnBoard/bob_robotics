#pragma once

// C++ includes
#include <functional>

// GeNN robotics includes
#include "../hid/joystick.h"

// local includes
#include "bebop.h"

namespace GeNNRobotics {
namespace Robots {
using ButtonEvent = std::function<bool(HID::Event &js)>;

class BebopJoystick : HID::Joystick
{
public:
    ButtonEvent m_ButtonCallback = nullptr;
    BebopJoystick(Bebop *bebop);

private:
    Bebop *m_Bebop;
    static constexpr float maxbank = 50;
    static constexpr float maxup = 50;
    static constexpr float maxyaw = 100;

    void onButtonEvent(HID::Event &js);
    void onAxisEvent(HID::Event &js);
    void eventCallback(HID::Event &js);
}; // BebopJoystick
}  // Robots
}  // GeNNRobotics
