#pragma once

// BoB robotics includes
#include "joystick_base.h"

// Linux includes
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

// Standard C includes
#include <cmath>
#include <cstdint>
#include <cstring>

// Standard C++ includes
#include <array>
#include <iostream>
#include <limits>
#include <thread>

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

/*!
 * \brief Class for reading from joysticks on Linux.
 *
 * *NOTE*: This class should not be used directly; see example in joystick_test.
 */
class JoystickLinux : public JoystickBase<JAxisLinux, JButtonLinux>
{
public:
    //! Open default joystick device with (optionally) specified dead zone
    JoystickLinux(float deadZone = 0.0f)
      : JoystickBase(deadZone)
    {
        // open joystick device
        m_Fd = ::open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        if (m_Fd < 0) {
            throw std::runtime_error("Could not open joystick");
        }

        // get initial states
        js_event event;
        while (read(event) && event.type & JS_EVENT_INIT) {
            if (event.type & JS_EVENT_AXIS) {
                const JAxis axis = toAxis(event.number);
                setState(axis, axisToFloat(axis, event.value), true);
            }
            else if (event.type & JS_EVENT_BUTTON) {
                setState(toButton(event.number),
                         event.value ? StateDown : 0, true);
            }
        }
    }

    //! Close connection to controller
    ~JoystickLinux()
    {
        ::close(m_Fd);
    }

protected:
    //------------------------------------------------------------------------
    // JoystickBase virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() override
    {
        // see if a new event is in buffer
        js_event event;
        if (!read(event)) {
            return false;
        }

        do {
            const bool isInitial = (event.type & JS_EVENT_INIT);

            if (event.type == JS_EVENT_AXIS) {
                const JAxis axis = toAxis(event.number);
                setState(axis, axisToFloat(axis, event.value), isInitial);
            } else {
                if (event.value) {
                    setPressed(toButton(event.number), isInitial);

                } else {
                    setReleased(toButton(event.number), isInitial);
                }
            }
        } while (read(event)); // read all events in buffer
        return true;
    }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    bool read(js_event &event)
    {
        ssize_t bytes;
        do {
            bytes = ::read(m_Fd, &event, sizeof(js_event));
            if (bytes == -1 && errno != EAGAIN) {
                throw std::runtime_error("Error reading from joystick (" +
                                         std::to_string(errno) + std::string(": ") +
                                         std::strerror(errno) + ")");
            }
            // ignore D-pad button events; handled as axis events
        } while (bytes > 0 && (event.type & JS_EVENT_BUTTON) && event.number > 10);

        return bytes > 0;
    }

    //------------------------------------------------------------------------
    // Static methods
    //------------------------------------------------------------------------
    //! Convert a raw 16-bit int value for an axis to a float
    static constexpr float axisToFloat(JAxis axis, int16_t value)
    {
        switch (axis) {
        case JAxis::LeftStickHorizontal:
        case JAxis::LeftStickVertical:
        case JAxis::RightStickHorizontal:
        case JAxis::RightStickVertical:
            return value > 0 ? static_cast<float>(value) / int16_maxf
                             : static_cast<float>(value) / int16_absminf;
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

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    int m_Fd = -1;      // file descriptor for joystick device
}; // JoystickLinux
} // HID
} // BoBRobotics
