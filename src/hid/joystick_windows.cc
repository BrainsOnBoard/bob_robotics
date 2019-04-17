#ifdef _WIN32
#include "joystick_base.inc"

// BoB robotics includes
#include "hid/joystick_windows.h"

// Standard C includes
#include <cstdint>
#include <fcntl.h>
#include <sys/stat.h>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

namespace {
float
getDpadValue(WORD buttons)
{
    return buttons ? (buttons == 1 ? -1.0f : 1.0f) : 0.0f;
}

void
read(XINPUT_STATE &state)
{
    // Zeroise the state
    ZeroMemory(&state, sizeof(state));

    // Get the state
    DWORD result = XInputGetState(0, &state);
    if (result != ERROR_SUCCESS) {
        throw std::runtime_error("Could not read from joystick");
    }
}
}

namespace BoBRobotics {
namespace HID {

JoystickWindows::JoystickWindows(float deadZone)
  : JoystickBase(deadZone)
  , m_LastPacketNumber(-1)
{
    // Read XInput state
    XINPUT_STATE state;
    read(state);

    // Set initial button states
    for (size_t i = toIndex(JButton::Start); i < toIndex(JButton::LENGTH); i++) {
        setState(toButton(i), ((state.Gamepad.wButtons >> i) & 1) ? StateDown : 0, true);
    }

    // Set initial axis states
    updateAxes(state, true);
}

JoystickWindows::~JoystickWindows()
{
    stop();
}

//------------------------------------------------------------------------
// JoystickBase virtuals
//------------------------------------------------------------------------
bool
JoystickWindows::updateState()
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
        const bool down = (((state.Gamepad.wButtons >> i) & 1) != 0);
        if (down != isDown(toButton(i))) {
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

//------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------
void
JoystickWindows::updateAxes(const XINPUT_STATE &state, bool isInitial)
{
    // Thumbsticks
    setState(JAxis::LeftStickHorizontal, axisToFloat(JAxis::LeftStickHorizontal, state.Gamepad.sThumbLX), isInitial);
    setState(JAxis::LeftStickVertical, axisToFloat(JAxis::LeftStickVertical, state.Gamepad.sThumbLY), isInitial);
    setState(JAxis::RightStickHorizontal, axisToFloat(JAxis::RightStickHorizontal, state.Gamepad.sThumbRX), isInitial);
    setState(JAxis::RightStickVertical, axisToFloat(JAxis::RightStickVertical, state.Gamepad.sThumbRY), isInitial);

    // Triggers
    setState(JAxis::LeftTrigger, axisToFloat(JAxis::LeftTrigger, static_cast<int16_t>(state.Gamepad.bLeftTrigger)), isInitial);
    setState(JAxis::RightTrigger, axisToFloat(JAxis::RightTrigger, static_cast<int16_t>(state.Gamepad.bRightTrigger)), isInitial);

    // D-pad
    setState(JAxis::DpadVertical, getDpadValue(state.Gamepad.wButtons & 3), isInitial);
    setState(JAxis::DpadHorizontal, getDpadValue((state.Gamepad.wButtons >> 2) & 3), isInitial);
}

//------------------------------------------------------------------------
// methods
//------------------------------------------------------------------------
constexpr float
JoystickWindows::axisToFloat(JAxis axis, int16_t value)
{
    switch (axis) {
    case JAxis::LeftStickHorizontal:
    case JAxis::RightStickHorizontal:
        return value < 0 ? static_cast<float>(value) / int16_absminf()
                         : static_cast<float>(value) / int16_maxf();
    case JAxis::LeftStickVertical:
    case JAxis::RightStickVertical:
        return value < 0 ? static_cast<float>(-value) / int16_absminf()
                         : static_cast<float>(-value) / int16_maxf();
    case JAxis::LeftTrigger:
    case JAxis::RightTrigger:
        return static_cast<float>(value) / 255.0f;
    default:
        return static_cast<float>(value);
    }
}
} // HID
} // BoBRobotics
#endif // _WIN32
