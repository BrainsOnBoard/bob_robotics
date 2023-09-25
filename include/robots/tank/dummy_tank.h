#pragma once

// BoB robotics includes
#include "robots/tank/tank_base.h"

// Third-party includes
#include "plog/Log.h"

namespace BoBRobotics {
namespace Robots {
namespace Tank {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Tank::DummyTank
//----------------------------------------------------------------------------
//! A tank interface which just prints out commands (for debugging)
class DummyTank : public TankBase<DummyTank>
{
    friend TankBase<DummyTank>;
public:
    static constexpr auto getRobotWidth()
    {
        return units::length::meter_t{ 1 };
    }

    static constexpr auto getMaximumSpeed()
    {
        return units::velocity::meters_per_second_t{ 1 };
    }
private:
    static void tankInternal(float left, float right)
    {
        LOGV << "Driving: " << left << ", " << right;
    }
}; // DummyTank
} // Tank
} // Robots
} // BoBRobotics
