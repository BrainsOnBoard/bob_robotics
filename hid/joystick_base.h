#pragma once

// C includes
#include <cmath>
#include <cstdint>

// C++ includes
#include <array>
#include <functional>
#include <limits>
#include <string>
#include <vector>

// BoB robotics includes
#include "../common/threadable.h"

namespace BoBRobotics {
namespace HID {

// helper macros
#define toIndex(value) static_cast<size_t>(value)
#define toAxis(value) static_cast<JAxis>(value)
#define toButton(value) static_cast<JButton>(value)

// maximum values
#define int16_maxf static_cast<float>(std::numeric_limits<int16_t>::max())
#define int16_absminf -static_cast<float>(std::numeric_limits<int16_t>::min())

//! The current state of a joystick button
enum ButtonState
{
    StateDown = (1 << 0), //!< Whether the button is being pressed down
    StatePressed = (1 << 1), //!< Whether the button has been pressed since last update()
    StateReleased = (1 << 2) //!< Whether the button has been released since last update()
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
template<typename Joystick, typename JAxis, typename JButton>
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

private:
    std::vector<ButtonHandler> m_ButtonHandlers;
    std::vector<AxisHandler> m_AxisHandlers;
    std::array<float, toIndex(JAxis::LENGTH)> m_AxisState;
    float m_DeadZone;

public:
    /*!
     * \brief Try to read from the joystick
     * 
     * @return True if one or more events were read from joystick
     */
    virtual bool update() = 0;

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
        return m_AxisState[toIndex(axis)];
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
    static constexpr std::string getName(JAxis axis)
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
    static constexpr std::string getName(JButton button)
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
    std::array<unsigned char, toIndex(JButton::LENGTH)> m_ButtonState;

    JoystickBase(float deadZone = 0.0f)
      : m_DeadZone(deadZone)
    {}

    void raiseButtonEvent(JButton button, bool pressed)
    {
        for (auto handler : m_ButtonHandlers) {
            if (handler(button, pressed)) {
                break;
            }
        }
    }

    void raiseAxisEvent(JAxis axis, float value, bool isInitial)
    {
        auto &s = m_AxisState[toIndex(axis)];
        if (s != value) {
            s = value;

            // run handlers
            if (!isInitial) {
                for (auto handler : m_AxisHandlers) {
                    if (handler(axis, value)) {
                        break;
                    }
                }
            }
        }
    }

    void updateAxis(JAxis axis, int16_t value, bool isInitial)
    {
        raiseAxisEvent(axis, Joystick::axisToFloat(axis, value), isInitial);
    }

    void updateAxis(JAxis axisHorz, int16_t hvalue, int16_t vvalue, bool isInitial)
    {
        JAxis axisVert = toAxis(toIndex(axisHorz) + 1);
        const float x = Joystick::axisToFloat(axisHorz, hvalue);
        const float y = Joystick::axisToFloat(axisVert, vvalue);

        if (sqrt(x * x + y * y) < m_DeadZone) {
            raiseAxisEvent(axisHorz, 0.0f, isInitial);
            raiseAxisEvent(axisVert, 0.0f, isInitial);
        } else {
            raiseAxisEvent(axisHorz, x, isInitial);
            raiseAxisEvent(axisVert, y, isInitial);
        }
    }
}; // JoystickBase
} // HID
} // BoBRobotics
