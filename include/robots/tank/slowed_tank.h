#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "robots/tank/tank_base.h"

namespace BoBRobotics {
namespace Robots {
namespace Tank {
template<class TankType>
class SlowedTank
  : public TankBase<SlowedTank<TankType>>
{
public:
    template<class... Ts>
    SlowedTank(Ts&&... args)
      : m_Tank(std::forward<Ts>(args)...)
    {}

    void tank(float left, float right)
    {
        m_Tank.tank(left * m_MaximumSpeed, right * m_MaximumSpeed);
    }

    void setMaximumSpeedProportion(float speed)
    {
        BOB_ASSERT(speed >= 0.f && speed <= 1.f);
        m_MaximumSpeed = speed;
    }

    auto getMaximumSpeed() const
    {
        return m_MaximumSpeed * m_Tank.getMaximumSpeed();
    }

    auto getMaximumTurnSpeed() const
    {
        return m_MaximumSpeed * m_Tank.getMaximumTurnSpeed();
    }

    auto getRobotWidth() const
    {
        return m_Tank.getRobotWidth();
    }

private:
    TankType m_Tank;
    float m_MaximumSpeed = 1.f;
}; // SlowedTank
} // Tank
} // Robots
} // BoBRobotics
