#pragma once

// Windows includes
#include "os/windows_include.h"
#include <xinput.h>
#pragma comment(lib, "XInput.lib")

// BoB robotics includes
#include "joystick_base.h"

namespace BoBRobotics {
namespace HID {

class JoystickWindows;

//! Joystick is set to JoystickLinux on Linux and JoystickWindows on Windows
using Joystick = JoystickWindows;

//! Controller axes, including thumbsticks, triggers and D-pad (Windows)
enum class JAxisWindows
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

//! JAxis is set to JAxisLinux on Linux and JAxisWindows on Windows
using JAxis = JAxisWindows;

/*!
 * \brief Controller buttons (Windows)
 *
 * The left stick and right stick are also buttons (you can click them).
 */
enum class JButtonWindows
{
    Start = 4,
    Back,
    LeftStick,
    RightStick,
    LB,
    RB,
    A = 12,
    B,
    X,
    Y,
    LENGTH
};

//! JButton is set to JButtonLinux on Linux and JButtonWindows on Windows
using JButton = JButtonWindows;

/*!
 * \brief Class for reading from joysticks on Windows.
 *
 * *NOTE*: This class should not be used directly; see example in joystick_test.
 */
class JoystickWindows : public JoystickBase<JAxisWindows, JButtonWindows>
{
public:
    //! Open default joystick device with (optionally) specified dead zone
    JoystickWindows(float deadZone = 0.0f);

    virtual ~JoystickWindows() override;

    //------------------------------------------------------------------------
    // JoystickBase virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() override;

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void updateAxes(const XINPUT_STATE &state, bool isInitial);

    //------------------------------------------------------------------------
    // Static methods
    //------------------------------------------------------------------------
    //! Convert a raw 16-bit int value for an axis to a float
    static constexpr float axisToFloat(JAxis axis, int16_t value);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    DWORD m_LastPacketNumber;
}; // Joystick
} // HID
} // BoBRobotics
