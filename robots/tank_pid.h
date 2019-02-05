#pragma once

// BoB robotics includes
#include "../common/circstat.h"
#include "../common/stopwatch.h"
#include "tank.h"

// Third-party includes
#include "../third_party/units.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <limits>

namespace BoBRobotics {
namespace Robots {
class TankPID
{
    using radian_t = units::angle::radian_t;
    using second_t = units::time::second_t;

public:
    TankPID(float Kp, float Ki, float Kd, float averageSpeed = 0.5f)
      : m_Kp(Kp)
      , m_Ki(Ki)
      , m_Kd(Kd)
      , m_AverageSpeed(averageSpeed)
    {}

    void start()
    {
        m_Stopwatch.start();
        m_LastHeadingOffset = std::numeric_limits<double>::quiet_NaN();
    }

    template<typename PositionType, typename PoseType>
    void drive(Tank &robot, const PoseType &robotPose, const PositionType &goal)
    {
        const radian_t headingToGoal = units::math::atan2(goal.y() - robotPose.y(), goal.x() - robotPose.x());
        const double headingOffset = circularDistance(headingToGoal, robotPose.yaw()).value();

        if (!std::isnan(m_LastHeadingOffset)) {
            // Time since this function was last called
            const float dt = static_cast<second_t>(m_Stopwatch.lap()).value();

            const float p = m_Kp * headingOffset;
            const float i = m_Ki * (headingOffset + m_LastHeadingOffset * dt);
            const float d = m_Kd * (m_LastHeadingOffset - headingOffset) / dt;

            const float differential = p + i + d;
            const float v1 = std::min(1.f, std::max(0.f, m_AverageSpeed + differential));
            const float v2 = std::min(1.f, std::max(0.f, m_AverageSpeed - differential));
            if (differential >= 0.f) {
                robot.tank(v1, v2);
            } else {
                robot.tank(v2, v1);
            }
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