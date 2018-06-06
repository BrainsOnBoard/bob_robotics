#pragma once

// C++ includes
#include <functional>

// GeNN robotics includes
#include "../hid/joystick.h"

// local includes
#include "bebop.h"

namespace GeNNRobotics {
namespace Robots {
class BebopJoystick : HID::Joystick
{
public:
    BebopJoystick(Bebop *bebop);

private:
    Bebop *m_Bebop;
    static constexpr float MaxBank = 50; // maximum % of speed for pitch/rool
    static constexpr float MaxUp = 50; // maximum % of speed for up/down motion
    static constexpr float MaxYaw = 100; // maximum % of speed for yaw

    bool onAxisEvent(HID::JAxis axis, float value);
    bool onButtonEvent(HID::JButton button, bool pressed);
}; // BebopJoystick
}  // Robots
}  // GeNNRobotics
