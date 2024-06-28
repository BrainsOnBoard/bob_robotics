// BoB robotics includes
#include "hid/net/source.h"
#include "net/connection.h"

namespace BoBRobotics {
namespace HID {
namespace Net {

//----------------------------------------------------------------------------
// BoBRobotics::HID::Net::Source
//----------------------------------------------------------------------------
Source::Source(BoBRobotics::Net::Connection &connection)
:   m_Connection(connection)
{
    // Set initial state of all axes to 0.0f
    for(size_t a = 0; a < static_cast<size_t>(JAxis::LENGTH); a++) {
        setState(static_cast<JAxis>(a), 0.0f, true);
    }

    // Set initial state of all buttons to down
    for(size_t b = 0; b < static_cast<size_t>(JButton::LENGTH); b++) {
        setState(static_cast<JButton>(b), 0, true);
    }

    // Handle incoming JOY_AXIS commands
    m_Connection.setCommandHandler("JOY_AXIS",
                                   [this](auto&, const auto &command)
                                   {
                                       std::lock_guard<std::mutex> lock(m_EventMutex);

                                       if (command.size() != 3) {
                                           throw BoBRobotics::Net::BadCommandError();
                                       }

                                       // Parse event
                                       m_Events.emplace_back();
                                       m_Events.back().axisNotButton = true;
                                       m_Events.back().axis.axis = toAxis(std::stoul(command[1]));
                                       m_Events.back().axis.value = std::stof(command[2]);
                                   });

    // Handle incoming JOY_BUTTON commands
    m_Connection.setCommandHandler("JOY_BUTTON",
                                   [this](auto&, const auto &command)
                                   {
                                       std::lock_guard<std::mutex> lock(m_EventMutex);

                                       if (command.size() != 3) {
                                           throw BoBRobotics::Net::BadCommandError();
                                       }

                                       // Parse event
                                       m_Events.emplace_back();
                                       m_Events.back().axisNotButton = false;
                                       m_Events.back().button.button = toButton(std::stoul(command[1]));
                                       m_Events.back().button.pressed = (std::stoul(command[2]) != 0);
                                   });
}
//------------------------------------------------------------------------
Source::~Source()
{
    // Remove command handlers
    m_Connection.setCommandHandler("JOY_AXIS", nullptr);
    m_Connection.setCommandHandler("JOY_BUTTON", nullptr);
}
//------------------------------------------------------------------------
bool Source::updateState()
{
    std::lock_guard<std::mutex> lock(m_EventMutex);

    if(m_Events.empty()) {
        return false;
    }
    else {
        for(const auto &e : m_Events) {
            if (e.axisNotButton) {
                setState(e.axis.axis, e.axis.value, false);
            } else {
                if (e.button.pressed) {
                    setPressed(e.button.button, false);

                } else {
                    setReleased(e.button.button, false);
                }
            }
        }

        m_Events.clear();
        return true;
    }
}
} // Net
} // HID
} // BoBRobotics
