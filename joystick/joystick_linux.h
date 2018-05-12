#pragma once

#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

namespace Joystick {

/*
 * Controller buttons. The left stick and right stick are also buttons (you can
 * click them.)
 */
enum Button
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    Back = 6,
    Start = 7,
    XboxButton = 8,
    LeftStickButton = 9,
    RightStickButton = 10,
    Left = 11,
    Right = 12,
    Up = 13,
    Down = 14
};
}

#include "joystick_base.h"

namespace Joystick {
class Joystick : public JoystickBase
{
public:
    /*
     * Open connection to controller. Return true if connected successfully,
     * false otherwise.
     */
    bool open()
    {
        m_Fd = ::open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        return m_Fd >= 0;
    }

    /*
     * Close connection to controller.
     */
    void close()
    {
        JoystickBase::close();
        ::close(m_Fd);
    }

    /*
     * Read controller event into js struct. Returns true if read successfully,
     * false if an error occurs.
     */
    bool read(Event &js)
    {
        while (!m_Closing) {
            const ssize_t bytes = ::read(m_Fd, &m_JsEvent, sizeof(m_JsEvent));
            if (bytes > 0) {
                break;
            }
            if (errno != EAGAIN) {
                return false;
            }

            usleep(sleepmillis * 1000);
        }
        if (m_Closing) {
            return false;
        }

        js.isInitial = m_JsEvent.type & JS_EVENT_INIT;
        js.isAxis = (m_JsEvent.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS;
        js.number = m_JsEvent.number;

        // if it's an axis event for the left or right stick, account for
        // deadzone
        if (js.isAxis && js.number >= LeftStickHorizontal &&
            js.number <= RightStickVertical &&
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
};
}
