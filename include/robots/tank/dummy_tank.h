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
class DummyTank
  : public TankBase<DummyTank>
{
public:
    static void tank(float left, float right)
    {
        LOGI << "Driving: " << left << ", " << right;
    }

    static constexpr auto getRobotWidth()
    {
        return units::length::meter_t{ 1 };
    }

    static constexpr auto getMaximumSpeed()
    {
        return units::velocity::meters_per_second_t{ 1 };
    }
}; // DummyTank
} // Tank
} // Robots
} // BoBRobotics
