#pragma once

// BoB robotics includes
#include "net/connection.h"
#include "tank.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <limits>

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::TankNetSink
//----------------------------------------------------------------------------
//! An interface for transmitting tank steering commands over the network
class TankNetSink : public Tank
{
private:
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

    Net::Connection &m_Connection;
    millimeter_t m_AxisLength{ std::numeric_limits<double>::quiet_NaN() };
    mutable meters_per_second_t m_ForwardSpeed{ std::numeric_limits<double>::quiet_NaN() };
    mutable radians_per_second_t m_TurnSpeed{ std::numeric_limits<double>::quiet_NaN() };
    float m_OldLeft = 0, m_OldRight = 0;

public:
    TankNetSink(Net::Connection &connection);

    virtual ~TankNetSink() override;

    virtual void setMaximumSpeedProportion(float value) override;

    //! Motor command: send TNK command over TCP
    virtual void tank(float left, float right) override;

    virtual millimeter_t getRobotWidth() const override;

    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override;

    virtual radians_per_second_t getAbsoluteMaximumTurnSpeed() const override;
}; // TankNetSink
} // Robots
} // BoBRobotics
