#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "common/circstat.h"
#include "common/pose.h"
#include "robots/tank.h"
#include "positioner.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

// Forward declaration
template<class PoseGetterType>
class RobotPositioner;

template<class PoseGetterType, class... Args>
auto createRobotPositioner(Robots::Tank &tank,
                           PoseGetterType &poseGetter,
                           Args&&... otherArgs)
{
    return RobotPositioner<PoseGetterType>{ tank, poseGetter, std::forward<Args>(otherArgs)... };
}

/*!
 * \brief A tool for moving a tank robot to a specified position in space
 *
 * This class requires that we have a mechanism for obtaining the robot's pose,
 * with, e.g. a Vicon system (see Vicon::UDPClient). The algorithm used is drawn
 * from: https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
 */
template<class PoseGetterType>
class RobotPositioner
  : public PositionerBase<RobotPositioner<PoseGetterType>>
{
    using meter_t = units::length::meter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radian_t = units::angle::radian_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

private:
    // Hardware
    Robots::Tank &m_Tank;
    PoseGetterType &m_PoseGetter;

    // Robot variables
    Pose2<meter_t, radian_t> m_RobotPose;
    Pose2<meter_t, radian_t> m_GoalPose;
    meter_t m_DistanceToGoal; // Euclidean distance from goal coordinate
    radian_t m_HeadingToGoal; // bearing (angle) from goal coordinate

    // user variables
    const meter_t m_StoppingDistance;     // if the robot's distance from goal < stopping dist, robot stops
    const radian_t m_AllowedHeadingError; // the amount of error allowed in the final heading
    const double m_K1;                    // curveness of the path to the goal
    const double m_K2;                    // speed of turning on the curves
    const double m_Alpha;                 // causes more sharply peaked curves
    const double m_Beta;                  // causes to drop velocity if 'k'(curveness) increases
    const meter_t m_StartSlowingAt;
    const float m_MinSpeed, m_MaxSpeed;
    bool m_Running = false;

    // updates the range and bearing from the goal location
    void updateRangeAndBearing()
    {
        const meter_t delta_x = m_GoalPose.x() - m_RobotPose.x();
        const meter_t delta_y = m_GoalPose.y() - m_RobotPose.y();

        // calculate distance
        m_DistanceToGoal = units::math::hypot(delta_x, delta_y);

        // calculate bearing
        m_HeadingToGoal = circularDistance(m_RobotPose.yaw(), units::math::atan2(delta_y, delta_x));

        // changing from <0,360> to <-180, 180>
        m_HeadingToGoal = normaliseAngle180(m_HeadingToGoal);
        m_RobotPose.yaw() = normaliseAngle180(m_RobotPose.yaw());
    }

    //-----------------PUBLIC API---------------------------------------------------------------------
public:
    RobotPositioner(
            Robots::Tank &tank,
            PoseGetterType &poseGetter,
            meter_t stoppingDistance,     // if the robot's distance from goal < stopping dist, robot stops
            radian_t allowedHeadingError, // the amount of error allowed in the final heading
            double k1,                    // curveness of the path to the goal
            double k2,                    // speed of turning on the curves
            double alpha,                 // causes more sharply peaked curves
            double beta,                  // causes to drop velocity if 'k'(curveness) increases
            meter_t startSlowingAt = 0_m,
            float robotMinSpeed = 0.2f,
            float robotMaxSpeed = 1.f)
      : m_Tank(tank)
      , m_PoseGetter(poseGetter)
      , m_StoppingDistance(stoppingDistance)
      , m_AllowedHeadingError(allowedHeadingError)
      , m_K1(k1)
      , m_K2(k2)
      , m_Alpha(alpha)
      , m_Beta(beta)
      , m_StartSlowingAt(startSlowingAt)
      , m_MinSpeed(robotMinSpeed)
      , m_MaxSpeed(robotMaxSpeed)
    {
        BOB_ASSERT(stoppingDistance >= 0_m);
        BOB_ASSERT(allowedHeadingError >= 0_rad);
        BOB_ASSERT(startSlowingAt >= 0_m);
        BOB_ASSERT(robotMaxSpeed >= 0.f && robotMaxSpeed <= 1.f);
        BOB_ASSERT(robotMinSpeed >= 0.f && robotMinSpeed <= 1.f);
    }

    void moveTo(const Pose2<meter_t, radian_t> &pose)
    {
        m_GoalPose = pose;

        // Recompute heading and distance from goal
        updateRangeAndBearing();
    }

    const auto &getRobot() const { return m_Tank; }
    auto &getRobot() { return m_Tank; }

    const auto &getPose() const
    {
        return m_RobotPose;
    }

    const auto &getGoalPose() const
    {
        return m_GoalPose;
    }

    auto distanceToGoal() const
    {
        return m_DistanceToGoal;
    }

    void reset()
    {
        m_Running = false;
    }

    bool pollPositioner()
    {
        // Update robot's position
        m_RobotPose = m_PoseGetter.getPose();
        updateRangeAndBearing();

        // Calculate velocities for new course
        meters_per_second_t v;
        radians_per_second_t omega;
        updateVelocities(v, omega);

        /*
         * Drive robot with specified velocities.
         * We invert the turning direction because we're counting anti-clockwise
         * for the robot's pose, but the Tank interface turns robots clockwise.
         * If the motor commands are out of range, they will be scaled down.
         */
        m_Tank.move(v, -omega, true);

        return !reachedGoal();
    }

    //! updates the velocities in order to get to a goal location. This function can be used
    //! without a robot interface, where only velocities are calculated but no robot actions
    //! will be executed.
    void updateVelocities(
            meters_per_second_t &v,      // velocity to update
            radians_per_second_t &omega) // angular velocity to update
    {
        // Set flag to show we've updated at least once
        m_Running = true;

        /*
         * Special case: if we're exactly on the goal, but at the wrong heading,
         * rotate on the spot. (We should only see this in simulation.)
         */
        if (m_DistanceToGoal == 0_m) {
            v = 0_mps;
            const radians_per_second_t maxTurnSpeed = m_Tank.getMaximumTurnSpeed();
            omega = (m_HeadingToGoal < 0_rad) ? -maxTurnSpeed : maxTurnSpeed;
            return;
        }

        // Extra logic for slowing the robot as it approaches the goal
        if (m_DistanceToGoal < m_StartSlowingAt) {
            const auto speedRange = m_MaxSpeed - m_MinSpeed;
            const auto speedProp = speedRange * m_DistanceToGoal / m_StartSlowingAt;
            m_Tank.setMaximumSpeedProportion(m_MinSpeed + speedProp);
        }

        // orientation of Target with respect to the line of sight from the observer to the target
        radian_t theta = circularDistance(m_GoalPose.yaw(), m_HeadingToGoal);

        using namespace units::dimensionless; // scalar_t
        const radian_t delta = units::math::atan(scalar_t((-m_K1 * theta).value()));
        const radian_t part1 = m_K2 * (m_HeadingToGoal - delta);
        const radian_t part2 = (1_rad + radian_t{ (m_K1 / (1 +
                                                           units::math::pow<2>(m_K1 * theta).value())) }) *
                               units::math::sin(m_HeadingToGoal);

        const auto k = -(part1 + part2) / m_DistanceToGoal; // in rad/mm

        v = m_Tank.getMaximumSpeed() / scalar_t((1 + m_Beta * pow(std::abs(k.value()), m_Alpha)));
        omega = k * v;
    }

    //! returns true if the robot reached the goal position
    bool reachedGoal() const
    {
        return m_Running && m_DistanceToGoal < m_StoppingDistance &&
               units::math::abs(circularDistance(m_RobotPose.yaw(), m_GoalPose.yaw())) < m_AllowedHeadingError;
    }

}; // RobotPositioner
} // Robots
} // BoBRobotics
