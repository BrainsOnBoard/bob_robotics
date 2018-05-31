#pragma once

// C includes
#include <cstdint>

// Linux includes
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

// C++ includes
#include <iostream>

namespace GeNNRobotics {
namespace HID {

/*
 * Controller buttons. The left stick and right stick are also buttons (you can
 * click them.)
 */
enum class Button
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
    NOTBUTTON
};
} // HID
} // GeNNRobotics

#include "joystick_base.h"

namespace GeNNRobotics {
namespace HID {
class Joystick : public JoystickBase
{
public:
    /*
     * Open connection to controller. Return true if connected successfully,
     * false otherwise.
     */
    Joystick()
    {
        open();
    }

    Joystick(const JoystickHandler handler) : JoystickBase(handler)
    {
        open();
    }

    /*
     * Close connection to controller.
     */
    ~Joystick()
    {
        ::close(m_Fd);
    }

    /*
     * Read controller event into js struct. Returns true if read successfully,
     * false if an error occurs.
     */
    bool read(Event &js) override
    {
        while (m_DoRun) {
            const ssize_t bytes = ::read(m_Fd, &m_JsEvent, sizeof(m_JsEvent));
            if (bytes > 0) {
                break;
            }
            if (errno != EAGAIN) {
                return false;
            }

            usleep(sleepmillis * 1000);
        }
        if (!m_DoRun) {
            return false;
        }

        js.isInitial = m_JsEvent.type & JS_EVENT_INIT;
        js.isAxis = (m_JsEvent.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS;
        js.number = m_JsEvent.number;

        // if it's an axis event for the left or right stick, account for
        // deadzone
        if (js.isAxis && js.number >= (uint) Axis::LeftStickHorizontal &&
            js.number <= (uint) Axis::RightStickVertical &&
            abs(m_JsEvent.value) < deadzone) {
            js.value = 0;
        } else {
            js.value = m_JsEvent.value;
        }

        return true;
    }

private:
    int m_Fd = 0;       // file descriptor for joystick device
    js_event m_JsEvent; // struct to contain joystick event
    static const int16_t deadzone = 10000; // size of deadzone for axes (i.e.
                                           // region within which not activated)
    static const long sleepmillis = 25; // number of milliseconds between polls

    void open()
    {
        m_Fd = ::open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        if (m_Fd < 0) {
            throw std::runtime_error("Could not open joystick");
        }
    }
}; // Joystick
} // HID
} // GeNNRobotics
