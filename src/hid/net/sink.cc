// BoB robotics includes
#include "hid/net/sink.h"
#include "net/connection.h"

// Standard C++ includes
#include <sstream>

namespace BoBRobotics {
namespace HID {
namespace Net {

Sink::Sink(BoBRobotics::Net::Connection &connection, Joystick &joystick)
: m_Connection{ connection }
{
    // Add axis handler to send network event
    joystick.addHandler(
        [this](auto&, JAxis axis, float value)
        {
            std::stringstream ss;
            ss << "JOY_AXIS " << static_cast<size_t>(axis) << " " << value << "\n";

            m_Connection.getSocketWriter().send(ss.str());
            return true;
        });

    // Add button handler to send network event
    joystick.addHandler(
        [this](auto&, JButton button, bool pressed)
        {
            std::stringstream ss;
            ss << "JOY_BTN " << static_cast<size_t>(button) << " " << (pressed ? "1" : "0") << "\n";

            m_Connection.getSocketWriter().send(ss.str());
            return true;
        });
}
} // Net
} // HID
} // BoBRobotics
