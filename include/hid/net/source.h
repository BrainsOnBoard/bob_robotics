#pragma once

// BoB robotics includes
#include "hid/joystick.h"

// Standard C++ includes
#include <mutex>
#include <vector>

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
// BoBRobotics::HID::Net::Source
//----------------------------------------------------------------------------
//! A virtual joystick controlled over the network
class Source : public JoystickBase<JAxis, JButton>
{
public:
    Source(BoBRobotics::Net::Connection &connection);
    virtual ~Source();

    //------------------------------------------------------------------------
    // JoystickBase virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() override final;

private:
    //------------------------------------------------------------------------
    // Event
    //------------------------------------------------------------------------
    //! Missing C++17 std::variant here...
    struct Event
    {
        bool axisNotButton;
        union
        {
            struct
            {
                JAxis axis;
                float value;
            } axis;

            struct
            {
                JButton button;
                bool pressed;
            } button;
        };
    };

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    BoBRobotics::Net::Connection &m_Connection;
    std::vector<Event> m_Events;
    std::mutex m_EventMutex;
};

} // Net
} // HID
} // BoBRobotics
