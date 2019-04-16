#pragma once

// BoB robotics includes
#include "common/pose.h"
#include "robots/tank.h"

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Robots {

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
    const meter_t m_StoppingDistance;     // if the robot's distance from goal < stopping dist, robot stops
    const radian_t m_AllowedHeadingError; // the amount of error allowed in the final heading
    const double m_K1;                    // curveness of the path to the goal
    const double m_K2;                    // speed of turning on the curves
    const double m_Alpha;                 // causes more sharply peaked curves
    const double m_Beta;                  // causes to drop velocity if 'k'(curveness) increases
    bool m_Running = false;

    // updates the range and bearing from the goal location
    void updateRangeAndBearing();

    //-----------------PUBLIC API---------------------------------------------------------------------
public:
    RobotPositioner(
            meter_t stoppingDistance,     // if the robot's distance from goal < stopping dist, robot stops
            radian_t allowedHeadingError, // the amount of error allowed in the final heading
            double k1,                    // curveness of the path to the goal
            double k2,                    // speed of turning on the curves
            double alpha,                 // causes more sharply peaked curves
            double beta);                 // causes to drop velocity if 'k'(curveness) increases

    //! sets the goal pose (x, y, angle)
    void setGoalPose(const Pose2<meter_t, radian_t> &pose);

    //! updates the agent's current pose
    void setPose(const Pose2<meter_t, radian_t> &pose);

    void reset();

    //! updates the velocities in order to get to a goal location. This function can be used
    //! without a robot interface, where only velocities are calculated but no robot actions
    //! will be executed.
    void updateVelocities(
            const Tank &bot,
            meters_per_second_t &v,       // velocity to update
            radians_per_second_t &omega); // angular velocity to update

    //! This function will update the motors so it drives towards a previously set goal location
    void updateMotors(BoBRobotics::Robots::Tank &bot,
                      const Pose2<meter_t, radian_t> &pose);

    //! returns true if the robot reached the goal position
    bool reachedGoal() const;

}; // RobotPositioner
} // Robots
} // BoBRobotics
