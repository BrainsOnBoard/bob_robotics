#pragma once

// BoB robotics includes
#include "joystick_base.h"

// Linux includes
#include <linux/joystick.h>

/*
 * We need to undef these macros (defined in linux/joystick.h) because they
 * break OpenCV.
 */
#ifdef KEY_UP
#undef KEY_UP
#endif
#ifdef KEY_DOWN
#undef KEY_DOWN
#endif

namespace BoBRobotics {
namespace HID {

class JoystickLinux;

//! Joystick is set to JoystickLinux on Linux and JoystickWindows on Windows
using Joystick = JoystickLinux;

//! Controller axes, including thumbsticks, triggers and D-pad (Linux)
enum class JAxisLinux
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
using JAxis = JAxisLinux;

/*!
 * \brief Controller buttons (Linux)
 *
 * The left stick and right stick are also buttons (you can click them).
 */
enum class JButtonLinux
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

//! JButton is set to JButtonLinux on Linux and JButtonWindows on Windows
using JButton = JButtonLinux;

// Explicitly instantiate class
template class JoystickBase<JAxisLinux, JButtonLinux>;

/*!
 * \brief Class for reading from joysticks on Linux.
 *
 * *NOTE*: This class should not be used directly; see example in joystick_test.
 */
class JoystickLinux : public JoystickBase<JAxisLinux, JButtonLinux>
{
public:
    //! Open default joystick device with (optionally) specified dead zone
    JoystickLinux(float deadZone = 0.0f);

    //! Close connection to controller
    virtual ~JoystickLinux() override;

protected:
    //------------------------------------------------------------------------
    // JoystickBase virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() override;

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    bool read(js_event &event);

    //------------------------------------------------------------------------
    // Static methods
    //------------------------------------------------------------------------
    //! Convert a raw 16-bit int value for an axis to a float
    static constexpr float axisToFloat(JAxis axis, int16_t value);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    int m_Fd = -1;      // file descriptor for joystick device
}; // JoystickLinux
} // HID
} // BoBRobotics
