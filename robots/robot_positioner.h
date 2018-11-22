#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "../robots/tank.h"

// Third-party includes
#include "../third_party/units.h"

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

/*!
 * \brief A tool for moving a tank robot to a specified position in space
 *
 * This class requires that we have a mechanism for obtaining the robot's pose,
 * with, e.g. a Vicon system (see Vicon::UDPClient). The algorithm used is drawn
 * from: https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
 */
class RobotPositioner
{
    using meter_t = units::length::meter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radian_t = units::angle::radian_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

private:
    // Robot variables
    Pose2<meter_t, radian_t> m_RobotPose;
    Pose2<meter_t, radian_t> m_GoalPose;
    meter_t m_DistanceToGoal; // Euclidean distance from goal coordinate
    radian_t m_HeadingToGoal; // bearing (angle) from goal coordinate

    // user variables
    const meter_t m_StoppingDistance;          // if the robot's distance from goal < stopping dist, robot stops
    const radian_t m_AllowedHeadingError;      // the amount of error allowed in the final heading
    const meters_per_second_t m_MaxVelocity;   // max velocity
    const radians_per_second_t m_MaxTurnSpeed; // max turning velocity
    const double m_K1;                         // curveness of the path to the goal
    const double m_K2;                         // speed of turning on the curves
    const double m_Alpha;                      // causes more sharply peaked curves
    const double m_Beta;                       // causes to drop velocity if 'k'(curveness) increases

    static radian_t angleWrapAround(radian_t angle)
    {

        while (angle < 0_deg) {
            angle += 360_deg;
        }
        while (angle > 360_deg) {
            angle -= 360_deg;
        }
        if (angle > 180_deg) {
            angle -= 360_deg;
        }

        return angle;
    }

    // updates the range and bearing from the goal location
    void updateRangeAndBearing()
    {

        const meter_t delta_x = m_GoalPose.x() - m_RobotPose.x();
        const meter_t delta_y = m_GoalPose.y() - m_RobotPose.y();

        // calculate distance
        m_DistanceToGoal = units::math::hypot(delta_x, delta_y);

        // calculate bearing
        m_HeadingToGoal = m_RobotPose.yaw() - units::math::atan2(delta_y, delta_x);

        // changing from <0,360> to <-180, 180>
        m_HeadingToGoal = angleWrapAround(m_HeadingToGoal);
        m_RobotPose.yaw() = angleWrapAround(m_RobotPose.yaw());
    }

    //-----------------PUBLIC API---------------------------------------------------------------------
public:
    RobotPositioner(

            meter_t stoppingDistance,         // if the robot's distance from goal < stopping dist, robot stops
            radian_t allowedHeadingError,     // the amount of error allowed in the final heading
            double k1,                        // curveness of the path to the goal
            double k2,                        // speed of turning on the curves
            double alpha,                     // causes more sharply peaked curves
            double beta,                      // causes to drop velocity if 'k'(curveness) increases
            meters_per_second_t maxVelocity,  // max velocity
            radians_per_second_t maxTurnSpeed // max turning speed
            )
      : m_StoppingDistance(stoppingDistance)
      , m_AllowedHeadingError(allowedHeadingError)
      , m_MaxVelocity(maxVelocity)
      , m_MaxTurnSpeed(maxTurnSpeed)
      , m_K1(k1)
      , m_K2(k2)
      , m_Alpha(alpha)
      , m_Beta(beta)
    {}

    //! sets the goal pose (x, y, angle)
    void setGoalPose(const Pose2<meter_t, radian_t> &pose)
    {
        m_GoalPose = pose;
        updateRangeAndBearing();
    }

    //! updates the agent's current pose
    void setPose(const Pose2<meter_t, radian_t> &pose)
    {
        m_RobotPose = pose;

        // Recompute heading and distance from goal
        updateRangeAndBearing();
    }

    //! updates the velocities in order to get to a goal location. This function can be used
    //! without a robot interface, where only velocities are calculated but no robot actions
    //! will be executed.
    void updateVelocities(
            meters_per_second_t &v,      // velocity to update
            radians_per_second_t &omega) // angular velocity to update
    {
        // If we're already at the goal, then we're done
        if (didReachGoal()) {
            v = 0_mps;
            omega = 0_rad_per_s;
            return;
        }

        /*
         * Special case: if we're exactly on the goal, but at the wrong heading,
         * rotate on the spot. (We should only see this in simulation.)
         */
        if (m_DistanceToGoal == 0_m) {
            v = 0_mps;
            omega = (m_HeadingToGoal < 0_rad) ? -m_MaxTurnSpeed : m_MaxTurnSpeed;
            return;
        }

        // orientation of Target with respect to the line of sight from the observer to the target
        radian_t theta = m_GoalPose.yaw() - m_HeadingToGoal;
        theta = angleWrapAround(theta);

        using namespace units::dimensionless; // scalar_t
        const radian_t delta = units::math::atan(scalar_t((-m_K1 * theta).value()));
        const radian_t part1 = m_K2 * (m_HeadingToGoal - delta);
        const radian_t part2 = (1_rad + radian_t{ (m_K1 / (1 +
                                                           units::math::pow<2>(m_K1 * theta).value())) }) *
                               units::math::sin(m_HeadingToGoal);

        const auto k = -(part1 + part2) / m_DistanceToGoal; // in rad/mm

        v = m_MaxVelocity / scalar_t((1 + m_Beta * pow(std::abs(k.value()), m_Alpha)));
        omega = k * v;
    }

    //! This function will update the motors so it drives towards a previously set goal location
    void updateMotors(BoBRobotics::Robots::Tank &bot,
                      const Pose2<meter_t, radian_t> &pose)
    {
        setPose(pose);

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
        bot.move(v, -omega, true);
    }

    //! returns true if the robot reached the goal position
    bool didReachGoal()
    {
        return m_DistanceToGoal < m_StoppingDistance &&
               units::math::abs(m_RobotPose.yaw() - m_GoalPose.yaw()) < m_AllowedHeadingError;
    }

}; // RobotPositioner
} // Robots
} // BoBRobotics
