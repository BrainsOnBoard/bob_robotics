#pragma once

// BoB robotics includes
#include "joystick_base.h"

namespace BoBRobotics {
namespace HID {
//! Controller axes, including thumbsticks, triggers and D-pad (Dummy)
enum class JAxisDummy
{
    LeftStickHorizontal = 0,
    LeftStickVertical = 1,
    RightStickHorizontal = 3,
    RightStickVertical = 4,
    LeftTrigger = 2,
    RightTrigger = 5,
    DpadHorizontal = 6,
    DpadVertical = 7,
    LENGTH
};

//! JAxis is set to JAxisDummy on Dummy and JAxisWindows on Windows
using JAxis = JAxisDummy;

/*!
 * \brief Controller buttons (Dummy)
 *
 * The left stick and right stick are also buttons (you can click them).
 */
enum class JButtonDummy
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    Back = 6,
    Start = 7,
    Xbox = 8,
    LeftStick = 9,
    RightStick = 10,
    LENGTH
};

class JoystickDummy
  : public JoystickBase<JAxisDummy, JButtonDummy>
{
public:
    JoystickDummy(float) {}

    virtual bool updateState() override
    {
        return false;
    }
};

using JAxis = JAxisDummy;
using JButton = JButtonDummy;
using Joystick = JoystickDummy;
} // HID
} // BoBRobotics
