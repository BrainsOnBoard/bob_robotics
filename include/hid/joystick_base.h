#pragma once

// BoB robotics includes
#include "common/threadable.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <array>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace HID {
using namespace std::literals;

//! The current state of a joystick button
enum ButtonState
{
    StateDown       = (1 << 0), //!< Whether the button is being pressed down
    StatePressed    = (1 << 1), //!< Whether the button has been pressed since last update()
    StateReleased   = (1 << 2) //!< Whether the button has been released since last update()
};

/*!
 * \brief Generic joystick class for Xbox 360 controller, from which
 *        JoystickLinux and JoystickWindows classes inherit.
 *
 * The class provides the basic, platform-independent functionality required
 * by JoystickLinux and JoystickWindows.
 *
 * *NOTE*: This class should not be used directly; see example in joystick_test.
 */
template<typename JAxis, typename JButton>
class JoystickBase : public Threadable
{
    /*!
     * \brief A delegate to handle joystick button presses
     *
     * @param button Which button was pressed/released
     * @param pressed Whether button was pressed/released
     * @return True if the function has handled the event, false otherwise
     */
    using ButtonHandler = std::function<bool(JButton button, bool pressed)>;

    /*!
     * \brief A delegate to handle joystick axis events (i.e. moving joysticks)
     *
     * @param axis Which axis was pressed/released
     * @param value Value from -1.0f to 1.0f representing new position of axis
     * @return True if the function has handled the event, false otherwise
     */
    using AxisHandler = std::function<bool(JAxis axis, float value)>;

protected:
    template<class T>
    static constexpr size_t toIndex(T value) { return static_cast<size_t>(value); }

    template<class T>
    static constexpr JAxis toAxis(T value) { return static_cast<JAxis>(value); }

    template<class T>
    static constexpr JButton toButton(T value) { return static_cast<JButton>(value); }

    static constexpr float int16_maxf() { return static_cast<float>(std::numeric_limits<int16_t>::max()); }
    static constexpr float int16_absminf() { return -static_cast<float>(std::numeric_limits<int16_t>::min()); }

public:
    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    /*!
     * \brief Try to read from the joystick
     *
     * @return True if one or more events were read from joystick
     */
    bool update()
    {
        // unset Pressed and Released bits for buttons
        for (auto &s : m_ButtonState) {
            s &= StateDown;
        }

        return updateState();
    }

    //! Add a function to handle joystick axis events
    void addHandler(AxisHandler handler)
    {
        m_AxisHandlers.insert(m_AxisHandlers.begin(), handler);
    }

    //! Add a function to handle joystick button events
    void addHandler(ButtonHandler handler)
    {
        m_ButtonHandlers.insert(m_ButtonHandlers.begin(), handler);
    }

    //! Get the current value for a specified joystick axis
    float getState(JAxis axis) const
    {
        if(axis == JAxis::LeftStickHorizontal) {
            return getDeadZonedState(JAxis::LeftStickHorizontal, JAxis::LeftStickVertical);
        }
        else if(axis == JAxis::LeftStickVertical) {
            return getDeadZonedState(JAxis::LeftStickVertical, JAxis::LeftStickHorizontal);
        }
        else if(axis == JAxis::RightStickHorizontal) {
            return getDeadZonedState(JAxis::RightStickHorizontal, JAxis::RightStickVertical);
        }
        else if(axis == JAxis::RightStickVertical) {
            return getDeadZonedState(JAxis::RightStickVertical, JAxis::RightStickHorizontal);
        }
        else {
            return m_AxisState[toIndex(axis)];
        }
    }

    /*!
     * \brief Get the current state for a specified joystick button
     *
     * \see ButtonState
     */
    unsigned char getState(JButton button) const
    {
        return m_ButtonState[toIndex(button)];
    }

    //! Whether button is currently being pressed
    bool isDown(JButton button) const
    {
        return getState(button) & StateDown;
    }

    //! Whether button has been pressed since last update()
    bool isPressed(JButton button) const
    {
        return getState(button) & StatePressed;
    }

    //! Whether button has been released since last update()
    bool isReleased(JButton button) const
    {
        return getState(button) & StateReleased;
    }

    //! Get the name of a specified joystick axis
    static std::string getName(JAxis axis)
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

    //! Get the name of a specified joystick button
    static std::string getName(JButton button)
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

protected:
    JoystickBase(float deadZone = 0.0f)
      : m_DeadZone(deadZone)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() = 0;

    //------------------------------------------------------------------------
    // Threadable virtuals
    //------------------------------------------------------------------------
    virtual void runInternal() override
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
    void setPressed(JButton button, bool isInitial)
    {
        // Get state and set StateDown and StatePressed
        uint8_t state = getState(button);
        state |= (StateDown | StatePressed);

        // Set new state
        setState(button, state, isInitial);
    }

    void setReleased(JButton button, bool isInitial)
    {
        // Get state, clear StateDown and set StateReleased
        uint8_t state = getState(button);
        state &= ~StateDown;
        state |= StateReleased;

        // Set new state
        setState(button, state, isInitial);
    }

    void setState(JButton button, uint8_t state, bool isInitial)
    {
        // Set button state
        m_ButtonState[toIndex(button)] = state;

        if(!isInitial && (isPressed(button) || isReleased(button))) {
            for (auto handler : m_ButtonHandlers) {
                if (handler(button, isPressed(button))) {
                    break;
                }
            }
        }
    }

    void setState(JAxis axis, float value, bool isInitial)
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

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    float getDeadZonedState(JAxis axis, JAxis axisPerpendicular) const
    {
        const float state = m_AxisState[toIndex(axis)];

        // If deadzone is enabled
        if(m_DeadZone > 0.0f) {
            // If axis and perpendicular axis are within circular deadzone, return 0
            const float statePerpendicular = m_AxisState[toIndex(axisPerpendicular)];
            if (sqrt((state * state) + (statePerpendicular * statePerpendicular)) < m_DeadZone) {
                return 0.0f;
            }
        }

        // Return axis state
        return state;
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::array<uint8_t, toIndex(JButton::LENGTH)> m_ButtonState;
    std::vector<ButtonHandler> m_ButtonHandlers;
    std::vector<AxisHandler> m_AxisHandlers;
    std::array<float, toIndex(JAxis::LENGTH)> m_AxisState;
    const float m_DeadZone;
}; // JoystickBase
} // HID
} // BoBRobotics
