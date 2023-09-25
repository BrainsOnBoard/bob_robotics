#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "net/client.h"
#include "net/connection.h"
#include "robots/tank/tank_base.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <limits>

namespace BoBRobotics {
namespace Robots {
namespace Tank {
namespace Net {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Tank::Net::SinkBase
//----------------------------------------------------------------------------
//! An interface for transmitting tank steering commands over the network
template<class ConnectionType = BoBRobotics::Net::Connection &>
class SinkBase : public TankBase<SinkBase<ConnectionType>>
{
private:
    friend TankBase<SinkBase<ConnectionType>>;

    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

    ConnectionType m_Connection;
    millimeter_t m_AxisLength{ std::numeric_limits<double>::quiet_NaN() };
    mutable meters_per_second_t m_ForwardSpeed{ std::numeric_limits<double>::quiet_NaN() };
    mutable radians_per_second_t m_TurnSpeed{ std::numeric_limits<double>::quiet_NaN() };
    float m_OldLeft = 0, m_OldRight = 0;

public:
    template<class... Ts>
    SinkBase(Ts &&... args)
      : m_Connection{ std::forward<Ts>(args)... }
    {
        m_Connection.setCommandHandler("TNK_PARAMS", [this](auto &, const auto &command) {
            if (command.size() != 4) {
                throw BoBRobotics::Net::BadCommandError();
            }

            m_TurnSpeed = radians_per_second_t(stod(command[1]));
            m_ForwardSpeed = meters_per_second_t(stod(command[2]));
            m_AxisLength = millimeter_t(stod(command[3]));
        });

        /*
         * If we start running the connection on a separate thread before this,
         * there's a chance the TNK_PARAMS command won't be caught.
         */
        BOB_ASSERT(!m_Connection.isRunning());

        // Wait for command
        while (m_Connection.readNextCommand() != "TNK_PARAMS")
            ;
    }

    ~SinkBase();

    millimeter_t getRobotWidth() const;

    meters_per_second_t getMaximumSpeed() const;

    radians_per_second_t getMaximumTurnSpeed() const;

    auto &getConnection()
    {
        return m_Connection;
    }

protected:
    //! Motor command: send TNK command over TCP
    void tankInternal(float left, float right);

}; // SinkBase

//! Default type for controlling robots over network
using Sink = SinkBase<BoBRobotics::Net::Connection &>;

/**!
 * \brief Control a robot over the network, initialising the connection in the
 *        constructor.
 */
class BundledSink
  : public SinkBase<BoBRobotics::Net::Client>
{
public:
    BundledSink();
}; // BundledSink

} // Net
} // Tank
} // Robots
} // BoBRobotics
