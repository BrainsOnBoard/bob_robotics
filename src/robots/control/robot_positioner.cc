// BoB robotics includes
#include "common/circstat.h"
#include "robots/control/robot_positioner.h"

// Standard C includes
#include <cmath>

using namespace units::literals;

namespace BoBRobotics {
namespace Robots {

// updates the range and bearing from the goal location
void RobotPositioner::updateRangeAndBearing()
{

    const meter_t delta_x = m_GoalPose.x() - m_RobotPose.x();
    const meter_t delta_y = m_GoalPose.y() - m_RobotPose.y();

    // calculate distance
    m_DistanceToGoal = units::math::hypot(delta_x, delta_y);

    // calculate bearing
    m_HeadingToGoal = circularDistance(m_RobotPose.yaw(), units::math::atan2(delta_y, delta_x));

    // changing from <0,360> to <-180, 180>
    m_HeadingToGoal = normaliseAngle360(m_HeadingToGoal);
    m_RobotPose.yaw() = normaliseAngle360(m_RobotPose.yaw());
}

//-----------------PUBLIC API---------------------------------------------------------------------
RobotPositioner::RobotPositioner(
        meter_t stoppingDistance,     // if the robot's distance from goal < stopping dist, robot stops
        radian_t allowedHeadingError, // the amount of error allowed in the final heading
        double k1,                    // curveness of the path to the goal
        double k2,                    // speed of turning on the curves
        double alpha,                 // causes more sharply peaked curves
        double beta                   // causes to drop velocity if 'k'(curveness) increases
        )
    : m_StoppingDistance(stoppingDistance)
    , m_AllowedHeadingError(allowedHeadingError)
    , m_K1(k1)
    , m_K2(k2)
    , m_Alpha(alpha)
    , m_Beta(beta)
{}

//! sets the goal pose (x, y, angle)
void RobotPositioner::setGoalPose(const Pose2<meter_t, radian_t> &pose)
{
    m_GoalPose = pose;
    updateRangeAndBearing();
}

//! updates the agent's current pose
void RobotPositioner::setPose(const Pose2<meter_t, radian_t> &pose)
{
    m_RobotPose = pose;

    // Recompute heading and distance from goal
    updateRangeAndBearing();
}

void RobotPositioner::reset()
{
    m_Running = false;
}

//! updates the velocities in order to get to a goal location. This function can be used
//! without a robot interface, where only velocities are calculated but no robot actions
//! will be executed.
void RobotPositioner::updateVelocities(
        const Tank &bot,
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
        const radians_per_second_t maxTurnSpeed = bot.getMaximumTurnSpeed();
        omega = (m_HeadingToGoal < 0_rad) ? -maxTurnSpeed : maxTurnSpeed;
        return;
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

    v = bot.getMaximumSpeed() / scalar_t((1 + m_Beta * pow(std::abs(k.value()), m_Alpha)));
    omega = k * v;
}

//! This function will update the motors so it drives towards a previously set goal location
void RobotPositioner::updateMotors(BoBRobotics::Robots::Tank &bot,
                                    const Pose2<meter_t, radian_t> &pose)
{
    setPose(pose);

    // Calculate velocities for new course
    meters_per_second_t v;
    radians_per_second_t omega;
    updateVelocities(bot, v, omega);

    /*
     * Drive robot with specified velocities.
     * We invert the turning direction because we're counting anti-clockwise
     * for the robot's pose, but the Tank interface turns robots clockwise.
     * If the motor commands are out of range, they will be scaled down.
     */
    bot.move(v, -omega, true);
}

//! returns true if the robot reached the goal position
bool RobotPositioner::reachedGoal() const
{
    return m_Running && m_DistanceToGoal < m_StoppingDistance &&
            units::math::abs(circularDistance(m_RobotPose.yaw(), m_GoalPose.yaw())) < m_AllowedHeadingError;
}

} // Robots
} // BoBRobotics
