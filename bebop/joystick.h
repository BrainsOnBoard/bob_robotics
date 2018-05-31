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
    static constexpr float MaxBank = 50; // maximum % of speed for pitch/rool
    static constexpr float MaxUp = 50; // maximum % of speed for up/down motion
    static constexpr float MaxYaw = 100; // maximum % of speed for yaw

    bool onButtonEvent(HID::Event &js);
    bool onAxisEvent(HID::Event &js);
    bool eventCallback(HID::Event &js);
}; // BebopJoystick
}  // Robots
}  // GeNNRobotics
