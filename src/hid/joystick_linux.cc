#ifdef __linux__
// BoB robotics includes
#include "hid/joystick_linux.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <limits>
#include <stdexcept>

// Linux includes
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace BoBRobotics {
namespace HID {

//! Open default joystick device with (optionally) specified dead zone
JoystickLinux::JoystickLinux(float deadZone)
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
        } else if (event.type & JS_EVENT_BUTTON) {
            setState(toButton(event.number),
                     event.value ? StateDown : 0,
                     true);
        }
    }
}

//! Close connection to controller
JoystickLinux::~JoystickLinux()
{
    stop();
    ::close(m_Fd);
}

//------------------------------------------------------------------------
// JoystickBase virtuals
//------------------------------------------------------------------------
bool
JoystickLinux::updateState()
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

//------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------
bool
JoystickLinux::read(js_event &event)
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
constexpr float
JoystickLinux::axisToFloat(JAxis axis, int16_t value)
{
    switch (axis) {
    case JAxis::LeftStickHorizontal:
    case JAxis::LeftStickVertical:
    case JAxis::RightStickHorizontal:
    case JAxis::RightStickVertical:
        return value > 0 ? static_cast<float>(value) / int16_maxf()
                         : static_cast<float>(value) / int16_absminf();
    case JAxis::LeftTrigger:
    case JAxis::RightTrigger:
        return (static_cast<float>(value) + int16_absminf()) /
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
} // HID
} // BoBRobotics
#endif // linux
