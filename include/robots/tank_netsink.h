#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "net/client.h"
#include "net/connection.h"
#include "tank.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <limits>

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::TankNetSinkBase
//----------------------------------------------------------------------------
//! An interface for transmitting tank steering commands over the network
template<class ConnectionType = Net::Connection &>
class TankNetSinkBase : public Tank
{
private:
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
    TankNetSinkBase(Ts &&... args)
      : m_Connection{ std::forward<Ts>(args)... }
    {
        m_Connection.setCommandHandler("TNK_PARAMS", [this](Net::Connection &, const Net::Command &command) {
            if (command.size() != 5) {
                throw Net::BadCommandError();
            }

            m_TurnSpeed = radians_per_second_t(stod(command[1]));
            m_ForwardSpeed = meters_per_second_t(stod(command[2]));
            m_AxisLength = millimeter_t(stod(command[3]));
            Tank::setMaximumSpeedProportion(stof(command[4]));
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

    virtual ~TankNetSinkBase() override;

    virtual void setMaximumSpeedProportion(float value) override;

    //! Motor command: send TNK command over TCP
    virtual void tank(float left, float right) override;

    virtual millimeter_t getRobotWidth() const override;

    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override;

    virtual radians_per_second_t getAbsoluteMaximumTurnSpeed() const override;

    Net::Connection &getConnection()
    {
        return m_Connection;
    }

}; // TankNetSinkBase

// Explicitly instantiate
template class TankNetSinkBase<Net::Connection &>;

//! Default type for controlling robots over network
using TankNetSink = TankNetSinkBase<Net::Connection &>;

/**!
 * \brief Control a robot over the network, initialising the connection in the
 *        constructor.
 */
class BundledTankNetSink
  : public TankNetSinkBase<Net::Client>
{
public:
    BundledTankNetSink();
}; // BundledTankNetSink

} // Robots
} // BoBRobotics
