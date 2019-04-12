// BoB robotics includes
#include "hid/joystick_base.h"

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace HID {
using namespace std::literals;
//------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------
template<class JAxis, class JButton>
bool
JoystickBase<JAxis, JButton>::update()
{
    // unset Pressed and Released bits for buttons
    for (auto &s : m_ButtonState) {
        s &= StateDown;
    }

    return updateState();
}

template<class JAxis, class JButton>
void
JoystickBase<JAxis, JButton>::addHandler(AxisHandler handler)
{
    m_AxisHandlers.insert(m_AxisHandlers.begin(), handler);
}

template<class JAxis, class JButton>
void
JoystickBase<JAxis, JButton>::addHandler(ButtonHandler handler)
{
    m_ButtonHandlers.insert(m_ButtonHandlers.begin(), handler);
}

template<class JAxis, class JButton>
float
JoystickBase<JAxis, JButton>::getState(JAxis axis) const
{
    if (axis == JAxis::LeftStickHorizontal) {
        return getDeadZonedState(JAxis::LeftStickHorizontal, JAxis::LeftStickVertical);
    } else if (axis == JAxis::LeftStickVertical) {
        return getDeadZonedState(JAxis::LeftStickVertical, JAxis::LeftStickHorizontal);
    } else if (axis == JAxis::RightStickHorizontal) {
        return getDeadZonedState(JAxis::RightStickHorizontal, JAxis::RightStickVertical);
    } else if (axis == JAxis::RightStickVertical) {
        return getDeadZonedState(JAxis::RightStickVertical, JAxis::RightStickHorizontal);
    } else {
        return m_AxisState[toIndex(axis)];
    }
}

template<class JAxis, class JButton>
unsigned char
JoystickBase<JAxis, JButton>::getState(JButton button) const
{
    return m_ButtonState[toIndex(button)];
}

template<class JAxis, class JButton>
bool
JoystickBase<JAxis, JButton>::isDown(JButton button) const
{
    return getState(button) & StateDown;
}

template<class JAxis, class JButton>
bool
JoystickBase<JAxis, JButton>::isPressed(JButton button) const
{
    return getState(button) & StatePressed;
}

template<class JAxis, class JButton>
bool
JoystickBase<JAxis, JButton>::isReleased(JButton button) const
{
    return getState(button) & StateReleased;
}

template<class JAxis, class JButton>
std::string
JoystickBase<JAxis, JButton>::getName(JAxis axis)
{
    switch (axis) {
    case JAxis::LeftStickHorizontal:
        return "left stick horizontal";
    case JAxis::LeftStickVertical:
        return "left stick vertical";
    case JAxis::RightStickHorizontal:
        return "right stick horizontal";
    case JAxis::RightStickVertical:
        return "right stick vertical";
    case JAxis::LeftTrigger:
        return "left trigger";
    case JAxis::RightTrigger:
        return "right trigger";
    case JAxis::DpadHorizontal:
        return "D-pad horizontal";
    case JAxis::DpadVertical:
        return "D-pad vertical";
    default:
        return "(unknown)";
    }
}

template<class JAxis, class JButton>
std::string
JoystickBase<JAxis, JButton>::getName(JButton button)
{
    switch (button) {
    case JButton::A:
        return "A";
    case JButton::B:
        return "B";
    case JButton::X:
        return "X";
    case JButton::Y:
        return "Y";
    case JButton::LB:
        return "LB";
    case JButton::RB:
        return "RB";
    case JButton::Back:
        return "back";
    case JButton::Start:
        return "start";
    case JButton::LeftStick:
        return "left stick";
    case JButton::RightStick:
        return "right stick";
    default:
        return "(unknown)";
    }
}

template<class JAxis, class JButton>
JoystickBase<JAxis, JButton>::JoystickBase(float deadZone)
  : m_DeadZone(deadZone)
{}

//------------------------------------------------------------------------
// Threadable virtuals
//------------------------------------------------------------------------
template<class JAxis, class JButton>
void
JoystickBase<JAxis, JButton>::runInternal()
{
    while (isRunning()) {
        if (!update()) {
            std::this_thread::sleep_for(50ms);
        }
    }
}

//------------------------------------------------------------------------
// Protected methods
//------------------------------------------------------------------------
template<class JAxis, class JButton>
void
JoystickBase<JAxis, JButton>::setPressed(JButton button, bool isInitial)
{
    // Get state and set StateDown and StatePressed
    uint8_t state = getState(button);
    state |= (StateDown | StatePressed);

    // Set new state
    setState(button, state, isInitial);
}

template<class JAxis, class JButton>
void
JoystickBase<JAxis, JButton>::setReleased(JButton button, bool isInitial)
{
    // Get state, clear StateDown and set StateReleased
    uint8_t state = getState(button);
    state &= ~StateDown;
    state |= StateReleased;

    // Set new state
    setState(button, state, isInitial);
}

template<class JAxis, class JButton>
void
JoystickBase<JAxis, JButton>::setState(JButton button, uint8_t state, bool isInitial)
{
    // Set button state
    m_ButtonState[toIndex(button)] = state;

    if (!isInitial && (isPressed(button) || isReleased(button))) {
        for (auto handler : m_ButtonHandlers) {
            if (handler(button, isPressed(button))) {
                break;
            }
        }
    }
}

template<class JAxis, class JButton>
void
JoystickBase<JAxis, JButton>::setState(JAxis axis, float value, bool isInitial)
{
    // If the state's changed
    // **NOTE** this is more for XINPUT which doesn't raise events
    if (m_AxisState[toIndex(axis)] != value) {
        m_AxisState[toIndex(axis)] = value;

        if (!isInitial) {
            // Get state after deadzone is taken into account
            const float processedState = getState(axis);

            // run handlers
            for (auto handler : m_AxisHandlers) {
                if (handler(axis, processedState)) {
                    break;
                }
            }
        }
    }
}

//------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------
template<class JAxis, class JButton>
float
JoystickBase<JAxis, JButton>::getDeadZonedState(JAxis axis, JAxis axisPerpendicular) const
{
    const float state = m_AxisState[toIndex(axis)];

    // If deadzone is enabled
    if (m_DeadZone > 0.0f) {
        // If axis and perpendicular axis are within circular deadzone, return 0
        const float statePerpendicular = m_AxisState[toIndex(axisPerpendicular)];
        if (sqrt((state * state) + (statePerpendicular * statePerpendicular)) < m_DeadZone) {
            return 0.0f;
        }
    }

    // Return axis state
    return state;
}

} // HID
} // BoBRobotics
