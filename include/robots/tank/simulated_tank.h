#pragma once

// BoB robotics includes
#include "common/circstat.h"
#include "common/macros.h"
#include "common/pose.h"
#include "common/stopwatch.h"
#include "robots/tank/tank_base.h"

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Robots {
namespace Tank {
using namespace units::literals;

template<typename LengthUnit = units::length::millimeter_t,
         typename AngleUnit = units::angle::degree_t>
class SimulatedTank : public TankBase<SimulatedTank<LengthUnit, AngleUnit>>
{
    friend TankBase<SimulatedTank<LengthUnit, AngleUnit>>;
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    SimulatedTank(const meters_per_second_t maximumSpeed = 0.3_mps, const millimeter_t axisLength = 104_mm)
      : m_MaximumSpeed(maximumSpeed)
      , m_AxisLength(axisLength)
    {}

    millimeter_t getRobotWidth() const
    {
        return m_AxisLength;
    }

    const auto &getPose()
    {
        updatePose();
        return m_Pose;
    }

    meters_per_second_t getMaximumSpeed() const
    {
        return m_MaximumSpeed;
    }

    void setPose(const Pose2<LengthUnit, AngleUnit> &pose)
    {
        m_MoveStopwatch.start();
        m_Pose = pose;
    }

    bool moveTo(const Pose2<LengthUnit, AngleUnit> &pose)
    {
        setPose(pose);
        return true;
    }

protected:
    void tankInternal(float left, float right)
    {
        updatePose();
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);
        m_Left = left * m_MaximumSpeed;
        m_Right = right * m_MaximumSpeed;
    }

private:
    Pose2<LengthUnit, AngleUnit> m_Pose;
    Stopwatch m_MoveStopwatch;
    const meters_per_second_t m_MaximumSpeed;
    const millimeter_t m_AxisLength;
    meters_per_second_t m_Right{}, m_Left{};

    void updatePose()
    {
        using namespace units::angle;
        using namespace units::length;
        using namespace units::math;
        using namespace units::time;

        const second_t elapsed = m_MoveStopwatch.lap();
        if (m_Left == m_Right) {
            const LengthUnit dist = m_Left * elapsed;
            m_Pose.x() += dist * cos(m_Pose.yaw());
            m_Pose.y() += dist * sin(m_Pose.yaw());
        } else {
            const meter_t width = getRobotWidth();
            const meter_t turnRadius = (width * (m_Left + m_Right)) /
                                       (2 * (m_Left - m_Right));
            const double deltaAngle = (m_Right - m_Left) * elapsed / width;
            const radian_t newAngle = normaliseAngle180(m_Pose.yaw() + radian_t{ deltaAngle });
            m_Pose.x() -= turnRadius * (sin(newAngle) - sin(m_Pose.yaw()));
            m_Pose.y() += turnRadius * (cos(newAngle) - cos(m_Pose.yaw()));
            m_Pose.yaw() = newAngle;
        }
    }
}; // SimulatedTank
} // Tank
} // Robots
} // BoBRobotics
