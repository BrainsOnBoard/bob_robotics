#pragma once

// C includes
#include <cstdint>

// C++ includes
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

// GeNN robotics includes
#include "../common/threadable.h"

namespace GeNNRobotics {
namespace HID {
/*
 * Controller axes. Note that the triggers are axes as is the dpad. The dpad is
 * slightly strange in that it's also treated as buttons (i.e. pressing up gives
 * both a button event and an axis event).
 */
enum class Axis
{
    LeftStickHorizontal = 0,
    LeftStickVertical = 1,
    RightStickHorizontal = 3,
    RightStickVertical = 4,
    LeftTrigger = 2,
    RightTrigger = 5,
    DpadHorizontal = 6,
    DpadVertical = 7,
    NOTAXIS
};

struct Event
{
    unsigned int number;
    int16_t value;
    bool isAxis;
    bool isInitial;

    Axis axis() const
    {
        return !isAxis ? Axis::NOTAXIS : static_cast<Axis>(number);
    }

    std::string axisName() const
    {
        switch (axis()) {
        case Axis::LeftStickHorizontal:
            return "left stick horizontal";
        case Axis::LeftStickVertical:
            return "left stick vertical";
        case Axis::RightStickHorizontal:
            return "right stick horizontal";
        case Axis::RightStickVertical:
            return "right stick vertical";
        case Axis::LeftTrigger:
            return "left trigger";
        case Axis::RightTrigger:
            return "right trigger";
        case Axis::DpadHorizontal:
            return "D-pad horizontal";
        case Axis::DpadVertical:
            return "D-pad vertical";
        case Axis::NOTAXIS:
            return "(not axis)";
        default:
            return "(unknown)";
        }
    }

    float axisValue() const
    {
        if (isAxis) {
            return static_cast<float>(value) /
                    static_cast<float>(std::numeric_limits<int16_t>::max());
        } else {
            return std::numeric_limits<float>::quiet_NaN();
        }
    }

    Button button() const
    {
        return isAxis ? Button::NOTBUTTON : static_cast<Button>(number);
    }

    std::string buttonName() const
    {
        // these values should be defined before this header is included
        switch (button()) {
        case Button::A:
            return "A";
        case Button::B:
            return "B";
        case Button::X:
            return "X";
        case Button::Y:
            return "Y";
        case Button::LB:
            return "LB";
        case Button::RB:
            return "RB";
        case Button::Back:
            return "back";
        case Button::Start:
            return "start";
#ifndef _WIN32
        // this button is not available in Windows
        case Button::Xbox:
            return "Xbox";
#endif
        case Button::LeftStick:
            return "left stick";
        case Button::RightStick:
            return "right stick";
        case Button::Left:
            return "left";
        case Button::Right:
            return "right";
        case Button::Up:
            return "up";
        case Button::Down:
            return "down";
        case Button::NOTBUTTON:
            return "(not button)";
        default:
            return "(unknown)";        
        }
    }
};

// For callbacks when a controller event occurs (button is pressed, axis moves)
using JoystickHandler = std::function<bool(Event &)>;

class JoystickBase : public Threadable
{
private:
    std::vector<JoystickHandler> m_Handlers;
    Event m_JsEvent;

public:
    virtual bool read(Event &js) = 0;

    bool read()
    {
        return read(m_JsEvent);
    }

    void run() override
    {
        if (m_Handlers.size() == 0) {
            throw std::runtime_error("No handlers for joystick set");
        }

        while (read()) {
            for (auto handler : m_Handlers) {
                if (handler(m_JsEvent)) {
                    break;
                }
            }
        }

        // std::cerr << "Error reading from joystick" << std::endl;
    }

    void addHandler(const JoystickHandler handler)
    {
        m_Handlers.insert(m_Handlers.begin(), handler);
    }

protected:
    JoystickBase()
    {}

    JoystickBase(const JoystickHandler handler) : m_Handlers({ handler })
    {}
}; // JoystickBase
} // HID
} // GeNNRobotics
