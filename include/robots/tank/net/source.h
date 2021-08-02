#pragma once

// BoB robotics includes
#include "net/connection.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <sstream>
#include <stdexcept>
#include <string>

namespace BoBRobotics {
namespace Robots {
namespace Tank {
namespace Net {

template<class TankType>
class Source
{
public:
    Source(BoBRobotics::Net::Connection &connection, TankType &tank)
      : m_Connection{ connection }
      , m_Tank{ tank }
    {
        // Send robot parameters over network
        double maxTurnSpeed = NAN, maxForwardSpeed = NAN, axisLength = NAN;
        try {
            maxTurnSpeed = m_Tank.getAbsoluteMaximumTurnSpeed().value();
        } catch (std::runtime_error &) {
            // Then getMaximumTurnSpeed() isn't implemented
        }
        try {
            maxForwardSpeed = m_Tank.getAbsoluteMaximumSpeed().value();
        } catch (std::runtime_error &) {
            // Then getMaximumSpeed() isn't implemented
        }
        try {
            axisLength = m_Tank.getRobotWidth().value();
        } catch (std::runtime_error &) {
            // Then getRobotWidth() isn't implemented
        }

        std::stringstream ss;
        ss << "TNK_PARAMS "
           << maxTurnSpeed << " "
           << maxForwardSpeed << " "
           << axisLength << " "
           << m_Tank.getMaximumSpeedProportion() << "\n";
        connection.getSocketWriter().send(ss.str());

        // Handle incoming TNK commands
        connection.setCommandHandler("TNK",
                                     [this](auto &connection, const auto &command) {
                                         onTankCommandReceived(connection, command);
                                     });

        connection.setCommandHandler("TNK_MAX",
                                     [this](auto &, const auto &command) {
                                         m_Tank.setMaximumSpeedProportion(stof(command.at(1)));
                                         m_Tank.tank(m_Left, m_Right);
                                     });
    }

    ~Source()
    {
        // Remove command handlers
        m_Connection.setCommandHandler("TNK", nullptr);
        m_Connection.setCommandHandler("TNK_MAX", nullptr);
    }

private:
    BoBRobotics::Net::Connection &m_Connection;
    TankType &m_Tank;
    float m_Left = 0.f, m_Right = 0.f;

    void onTankCommandReceived(BoBRobotics::Net::Connection &,
                               const BoBRobotics::Net::Command &command)
    {
        // second space separates left and right parameters
        if (command.size() != 3) {
            throw BoBRobotics::Net::BadCommandError();
        }

        // parse strings to floats
        m_Left = std::stof(command[1]);
        m_Right = std::stof(command[2]);

        // send motor command
        m_Tank.tank(m_Left, m_Right);
    }
}; // Source

//! Helper factory function
template<class TankType>
auto
createSource(BoBRobotics::Net::Connection &connection, TankType &tank)
{
    return Source<TankType>{ connection, tank };
}

} // Net
} // Tank
} // Robots
} // BoBRobotics
