#pragma once

// Windows includes
#include "../os/windows_include.h"
#include <xinput.h>
#pragma comment(lib, "XInput.lib")

// C includes
#include <cstdint>
#include <fcntl.h>
#include <sys/stat.h>

// C++ includes
#include <chrono>
#include <iostream>
#include <thread>

// local includes
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
    JoystickWindows(float deadZone = 0.0f)
      : JoystickBase(deadZone), m_LastPacketNumber(DWORD_MAX)
    {
        // Read XInput state
        XINPUT_STATE state;
        read(state);

        // Set initial button states
        for (size_t i = toIndex(JButton::Start); i < toIndex(JButton::LENGTH); i++) {
            setState(toButton(i), ((m_State.Gamepad.wButtons >> i) & 1) ? StateDown : 0);
        }

        // Set initial axis states
        updateAxes(m_State, true);
    }

    //------------------------------------------------------------------------
    // JoystickBase virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() override
    {
        // Read XInput state
        XINPUT_STATE state;
        read(state);

        // Check that something has changed
        if (state.dwPacketNumber == m_LastPacketNumber) {
            return false;
        }

        // Update last packet number
        m_LastPacketNumber = state.dwPacketNumber;

        // Check buttons for changes
        for (size_t i = toIndex(JButton::Start); i < toIndex(JButton::LENGTH); i++) {
            const bool down = (((m_NewState.Gamepad.wButtons >> i) & 1) != 0);
            if (down != isDown(toButton(i)) {
                // ... then this button has been pressed or released
                if (down) {
                    setPressed(toButton(i), false);
                } else {
                    setReleased(toButton(i), false);
                }
            }
        }

        // Check axes for changes
        updateAxes(state, false);

        return true;
    }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void updateAxes(const XINPUT_STATE &state, bool isInitial)
    {
        // Thumbsticks
        setState(JAxis::LeftStickHorizontal, axisToFloat(JAxis::LeftStickHorizontal, state.Gamepad.sThumbLX), isInitial);
        setState(JAxis::LeftStickVertical, axisToFloat(JAxis::LeftStickVertical, state.Gamepad.sThumbLY), isInitial);
        setState(JAxis::RightStickHorizontal, axisToFloat(JAxis::RightStickHorizontal, state.Gamepad.sThumbRX), isInitial);
        setState(JAxis::RightStickVertical, axisToFloat(JAxis::RightStickVertical, state.Gamepad.sThumbRY), isInitial);

        // Triggers
        setState(JAxis::LeftTrigger, axisToFloat(JAxis::LeftTrigger, static_cast<int16_t>(state.Gamepad.bLeftTrigger)), isInitial);
        setState(JAxis::RightTrigger, axisToFloat(JAxis::RightTrigger, static_cast<int16_t>(state.Gamepad.bRightrigger)), isInitial);

        // D-pad
        setState(JAxis::DpadVertical, getDpadValue(state.Gamepad.wButtons & 3), isInitial);
        setState(JAxis::DpadHorizontal, getDpadValue((state.Gamepad.wButtons >> 2) & 3), isInitial);
    }

    //------------------------------------------------------------------------
    // Static methods
    //------------------------------------------------------------------------
    //! Convert a raw 16-bit int value for an axis to a float
    static constexpr float axisToFloat(JAxis axis, int16_t value) const override
    {
        switch (axis) {
        case JAxis::LeftStickHorizontal:
        case JAxis::RightStickHorizontal:
            return value < 0 ? static_cast<float>(value) / int16_absminf
                             : static_cast<float>(value) / int16_maxf;
        case JAxis::LeftStickVertical:
        case JAxis::RightStickVertical:
            return value < 0 ? static_cast<float>(-value) / int16_absminf
                             : static_cast<float>(-value) / int16_maxf;
        case JAxis::LeftTrigger:
        case JAxis::RightTrigger:
            return static_cast<float>(value) / 255.0f;
        default:
            return static_cast<float>(value);
        }
    }

    static float getDpadValue(WORD buttons)
    {
        return buttons ? (buttons == 1 ? -1.0f : 1.0f) : 0.0f;
    }

    static void read(XINPUT_STATE &state)
    {
        // Zeroise the state
        ZeroMemory(&state, sizeof(state));

        // Get the state
        DWORD result = XInputGetState(0, &state);
        if (result != ERROR_SUCCESS) {
            throw std::runtime_error("Could not read from joystick");
        }
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    DWORD m_LastPacketNumber;
}; // Joystick
} // HID
} // BoBRobotics
