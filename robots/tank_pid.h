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
#include <iostream>
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

        /*
         * We start off assuming that the heading is way off and that we need to
         * turn on the spot.
         */
        m_AdjustingHeading = true;
        std::cout << "Starting turning" << std::endl;
    }

    template<typename PositionType, typename PoseType>
    void drive(Tank &robot, const PoseType &robotPose, const PositionType &goal)
    {
        const radian_t headingToGoal = units::math::atan2(goal.y() - robotPose.y(), goal.x() - robotPose.x());
        const radian_t headingOffset = circularDistance(headingToGoal, robotPose.yaw());
        const float error = headingOffset.value();

        if (!std::isnan(m_LastHeadingOffset)) {
            // Time since this function was last called
            const float dt = static_cast<second_t>(m_Stopwatch.lap()).value();

            bool oldAdjusting = m_AdjustingHeading;
            const auto absError = units::math::abs(headingOffset);
            m_AdjustingHeading = (m_AdjustingHeading && absError > 3_deg) || absError > 90_deg;
            if (oldAdjusting != m_AdjustingHeading) {
                if (oldAdjusting) {
                    std::cout << "Stopping turning" << std::endl;
                } else {
                    std::cout << "Starting turning" << std::endl;
                }
            }
            if (m_AdjustingHeading) {
                // Hard-code a turning behaviour for when the heading is way out
                if (error < 0.f) {
                    robot.tank(1.f, -1.f);
                } else {
                    robot.tank(-1.f, 1.f);
                }
            } else {
                // PID controller for straight(ish) portion of course
                const float p = m_Kp * error;
                const float i = m_Ki * (error + m_LastHeadingOffset * dt);
                const float d = m_Kd * (m_LastHeadingOffset - error) / dt;

                const float differential = p + i + d;

                const float v1 = std::min(1.f, std::max(0.f, m_AverageSpeed + differential));
                const float v2 = std::min(1.f, std::max(0.f, m_AverageSpeed - differential));
                if (differential >= 0.f) {
                    robot.tank(v1, v2);
                } else {
                    robot.tank(v2, v1);
                }
            }
        }

        m_LastHeadingOffset = error;
    }

private:
    double m_LastHeadingOffset = std::numeric_limits<double>::quiet_NaN();
    Stopwatch m_Stopwatch;
    const float m_Kp, m_Ki, m_Kd, m_AverageSpeed;
    bool m_AdjustingHeading = true;
}; // TankPID
} // Robots
} // BoBRobotics