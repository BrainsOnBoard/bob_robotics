#pragma once

// C includes
#include <cstdint>

// C++ includes
#include <array>
#include <functional>
#include <string>
#include <vector>

// GeNN robotics includes
#include "../common/threadable.h"

namespace GeNNRobotics {
namespace HID {
/*
 * Controller axes, including thumbsticks, triggers and D-pad.
 */
enum class JAxis
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

enum ButtonState
{
    StateDown = (1 << 0),
    StatePressed = (1 << 1),
    StateReleased = (1 << 2)
};

template<typename Joystick, typename JButton>
class JoystickBase : public Threadable
{
    using ButtonHandler = std::function<bool(JButton button, bool pressed)>;
    using AxisHandler = std::function<bool(JAxis axis, float value)>;

private:
    std::vector<ButtonHandler> m_ButtonHandlers;
    std::vector<AxisHandler> m_AxisHandlers;

public:
    // Virtual methods
    virtual bool update() = 0;
    virtual float getAxisState(JAxis axis) const = 0;

    void addHandler(AxisHandler handler)
    {
        m_AxisHandlers.insert(m_AxisHandlers.begin(), handler);
    }

    void addHandler(ButtonHandler handler)
    {
        m_ButtonHandlers.insert(m_ButtonHandlers.begin(), handler);
    }

    unsigned char getButtonState(JButton button) const
    {
        return m_ButtonState[static_cast<size_t>(button)];
    }

    bool isButtonDown(JButton button) const
    {
        return getButtonState(button) & StateDown;
    }

    bool isButtonPressed(JButton button) const
    {
        return getButtonState(button) & StatePressed;
    }

    bool isButtonReleased(JButton button) const
    {
        return getButtonState(button) & StateReleased;
    }

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

    static std::string getName(JButton button)
    {
        // these values should be defined before this header is included
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
#ifndef _WIN32
        // this button is not available in Windows
        case JButton::Xbox:
            return "Xbox";
#endif
        case JButton::LeftStick:
            return "left stick";
        case JButton::RightStick:
            return "right stick";
        case JButton::Left:
            return "left";
        case JButton::Right:
            return "right";
        case JButton::Up:
            return "up";
        case JButton::Down:
            return "down";
        default:
            return "(unknown)";
        }
    }

protected:
    std::array<unsigned char, static_cast<unsigned long>(JButton::LENGTH)> m_ButtonState;

    JoystickBase()
    {}

    void raiseAxisEvent(JAxis axis, int16_t value)
    {
        float fvalue = Joystick::getAxisValue(axis, value);
        for (auto handler : m_AxisHandlers) {
            if (handler(axis, fvalue)) {
                break;
            }
        }
    }

    void raiseButtonEvent(JButton button, bool pressed)
    {
        for (auto handler : m_ButtonHandlers) {
            if (handler(button, pressed)) {
                break;
            }
        }
    }

}; // JoystickBase
} // HID
} // GeNNRobotics
