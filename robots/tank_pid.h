#pragma once

// BoB robotics includes
#include "../common/circstat.h"
#include "../common/stopwatch.h"
#include "tank.h"

// Third-party includes
#include "../third_party/units.h"

// Standard C++ includes
#include <limits>

namespace BoBRobotics {
namespace Robots {
class TankPID
{
    using radian_t = units::angle::radian_t;
    using second_t = units::time::second_t;

public:
    TankPID(float Kp, float Ki, float Kd, float averageSpeed = 1.f)
      : m_Kp(Kp)
      , m_Ki(Ki)
      , m_Kd(Kd)
      , m_AverageSpeed(averageSpeed)
    {}

    void start()
    {
        m_Stopwatch.start();
    }

    template<typename PositionType, typename PoseType>
    void drive(Tank &robot, const PoseType &robotPose, const PositionType &goal)
    {
        const radian_t headingToGoal = units::math::atan2(goal.y() - robotPose.y(), goal.x() - robotPose.x());
        const double headingOffset = circularDistance(headingToGoal, robotPose.yaw()).value();

        // Time since this function was last called
        const float dt = static_cast<second_t>(m_Stopwatch.lap()).value();

        const float p = m_Kp * headingOffset;
        const float i = m_Ki * (headingOffset + m_LastHeadingOffset * dt);
        const float d = m_Kd * (m_LastHeadingOffset - headingOffset) / dt;

        const float differential = std::max(-.5f, std::min(0.5f, p + i + d));
        if (differential >= 0.f) {
            robot.tank(m_AverageSpeed + differential, m_AverageSpeed - differential);
        } else {
            robot.tank(m_AverageSpeed - differential, m_AverageSpeed + differential);
        }

        m_LastHeadingOffset = headingOffset;
    }

private:
    double m_LastHeadingOffset = std::numeric_limits<double>::quiet_NaN();
    Stopwatch m_Stopwatch;
    const float m_Kp, m_Ki, m_Kd, m_AverageSpeed;
}; // TankPID
} // Robots
} // BoBRobotics