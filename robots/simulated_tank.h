#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "tank.h"

// Third-party includes
#include "../third_party/units.h"

// Standard C++ includes
#include <chrono>

namespace BoBRobotics {
namespace Robots {
template<typename LengthUnit, typename AngleUnit>
class SimulatedTank
  : public Tank
{
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    SimulatedTank(const meters_per_second_t maximumSpeed, const millimeter_t axisLength)
      : m_MaximumSpeed(maximumSpeed)
      , m_AxisLength(axisLength)
    {}

    virtual millimeter_t getRobotAxisLength()
    {
        return m_AxisLength;
    }

    const auto &getPose() const
    {
        return m_Pose;
    }

    meters_per_second_t getMaximumSpeed() const
    {
        return m_MaximumSpeed;
    }

    void setPose(const Pose2<LengthUnit, AngleUnit> &pose)
    {
        m_Pose = pose;
    }

    virtual void tank(float left, float right) override
    {
        updatePose();
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);
        m_Left = left * m_MaximumSpeed;
        m_Right = right * m_MaximumSpeed;
    }

private:
    Pose2<LengthUnit, AngleUnit> m_Pose;
    std::chrono::high_resolution_clock::time_point m_MoveStartTime;
    const meters_per_second_t m_MaximumSpeed;
    const millimeter_t m_AxisLength;
    meters_per_second_t m_Right{}, m_Left{};

    void updatePose()
    {
        using namespace units::angle;
        using namespace units::length;
        using namespace units::math;
        using namespace units::time;

        const auto currentTime = now();
        const second_t elapsed = currentTime - m_MoveStartTime;
        m_MoveStartTime = currentTime;

        if (m_Left == m_Right) {
            const LengthUnit dist = m_Left * elapsed;
            m_Pose.x += dist * cos(m_Pose.angle);
            m_Pose.y += dist * sin(m_Pose.angle);
        } else {
            const meter_t width = getRobotAxisLength();
            const auto turnRadius = (width * (m_Left + m_Right)) /
                                    (2 * (m_Left - m_Right));
            const double deltaAngle = (m_Right - m_Left) * elapsed / width;
            const radian_t newAngle = m_Pose.angle + radian_t{ deltaAngle };
            m_Pose.x += turnRadius * (sin(newAngle) - sin(m_Pose.angle));
            m_Pose.y += turnRadius * (cos(newAngle) - cos(m_Pose.angle));
            m_Pose.angle = newAngle;
        }
    }

    static auto now()
    {
        return std::chrono::high_resolution_clock::now();
    }
}; // SimulatedTank
} // Robots
} // BoBRobotics
