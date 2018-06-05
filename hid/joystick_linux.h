#pragma once

// C includes
#include <cstdint>
#include <cstring>

// C++ includes
#include <iostream>
#include <limits>
#include <thread>

// Linux includes
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

// local includes
#include "joystick_base.h"

namespace GeNNRobotics {
namespace HID {
/*
 * Controller buttons. The left stick and right stick are also buttons (you can
 * click them.)
 */
enum class JButton
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
    Left = 11,
    Right = 12,
    Up = 13,
    Down = 14,
    LENGTH
};

class Joystick : public JoystickBase<Joystick, JButton>
{
public:
    Joystick()
    {
        // open joystick device
        m_Fd = ::open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        if (m_Fd < 0) {
            throw std::runtime_error("Could not open joystick");
        }

        // get initial states
        while (read() && m_JsEvent.type & JS_EVENT_INIT) {
            if (m_JsEvent.type & JS_EVENT_AXIS) {
                setAxisState();
            } else {
                // initially set button values to either Down (1) or 0
                m_ButtonState[m_JsEvent.number] = m_JsEvent.value;
            }
        }
    }

    /*
     * Close connection to controller.
     */
    ~Joystick()
    {
        ::close(m_Fd);
    }

    virtual float getState(JAxis axis) const override
    {
        return m_AxisState[static_cast<size_t>(axis)];
    }

    virtual void run() override
    {
        while (m_DoRun) {
            while (!update()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }

    virtual bool update() override
    {
        // see if a new event is in buffer
        if (!read()) {
            return false;
        }

        if (m_JsEvent.type == JS_EVENT_AXIS) {
            // update current axis's state
            setAxisState();

            // run axis event handlers
            raiseEvent(static_cast<JAxis>(m_JsEvent.number), m_JsEvent.value);
        } else {
            // unset Pressed and Released bits for buttons
            for (auto &s : m_ButtonState) {
                s &= StateDown;
            }

            // update current button's state
            uint8_t &s = m_ButtonState[m_JsEvent.number];
            if (m_JsEvent.value) {
                s |= (StateDown | StatePressed); // set StateDown and StatePressed
            } else {                
                s &= ~StateDown;    // clear StateDown
                s |= StateReleased; // set StateReleased
            }

            // run button event handlers
            raiseEvent(static_cast<JButton>(m_JsEvent.number), m_JsEvent.value);
        }
        return true;
    }

    static float getAxisValue(JAxis axis, int16_t value)
    {
        switch (axis) {
        case JAxis::LeftStickHorizontal:
        case JAxis::LeftStickVertical:
        case JAxis::RightStickHorizontal:
        case JAxis::RightStickVertical:
            if (value > 0) {
                return static_cast<float>(value) / int16_maxf;
            } else {
                return static_cast<float>(value) / int16_absminf;
            }
        case JAxis::LeftTrigger:
        case JAxis::RightTrigger:
            return (static_cast<float>(value) + int16_absminf) /
                   static_cast<float>(std::numeric_limits<uint16_t>::max());
        case JAxis::DpadHorizontal:
        case JAxis::DpadVertical:
            switch (value) {
            case std::numeric_limits<int16_t>::max():
                return 1.0f;
            case 0:
                return 0.0f;
            default:
                return -1.0f;
            }
        default:
            return std::numeric_limits<float>::quiet_NaN();
        }
    }

private:
    int m_Fd = 0; // file descriptor for joystick device
    std::array<float, static_cast<unsigned long>(JAxis::LENGTH)> m_AxisState;
    js_event m_JsEvent; // struct to contain joystick event

    bool read()
    {
        ssize_t bytes;
        do {
            bytes = ::read(m_Fd, &m_JsEvent, sizeof(m_JsEvent));
            if (errno && errno != EAGAIN) {
                throw std::runtime_error("Error reading from joystick (" +
                                         std::to_string(errno) + std::string(": ") +
                                         std::strerror(errno) + ")");
            }
            // ignore D-pad button events; handled as axis events
        } while (m_JsEvent.type & JS_EVENT_BUTTON && m_JsEvent.number > 10);

        return bytes > 0;
    }

    void setAxisState()
    {
        auto axis = m_JsEvent.number;
        m_AxisState[axis] = getAxisValue(static_cast<JAxis>(axis), m_JsEvent.value);
    }

    static constexpr float int16_maxf = static_cast<float>(std::numeric_limits<int16_t>::max());
    static constexpr float int16_absminf = -static_cast<float>(std::numeric_limits<int16_t>::min());
}; // Joystick
} // HID
} // GeNNRobotics
