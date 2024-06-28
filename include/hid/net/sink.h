#pragma once

// BoB robotics includes
#include "hid/joystick.h"

// Forward declarations
namespace BoBRobotics {
namespace Net {
    class Connection;
}
}

namespace BoBRobotics {
namespace HID {
namespace Net {

//----------------------------------------------------------------------------
// BoBRobotics::HID::Net::Sink
//----------------------------------------------------------------------------
//! An interface for transmitting joystick commands over the network
class Sink
{
public:
    Sink(BoBRobotics::Net::Connection &connection, Joystick &joystick);

private:
    BoBRobotics::Net::Connection &m_Connection;
}; // Source


} // Net
} // HID
} // BoBRobotics
