#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "../robots/tank.h"

// Third-party includes
#include "../third_party/units.h"

// Standard C includes
#include <cmath>

#define PI 3.142

namespace BoBRobotics {
namespace Robots {

using namespace std::literals;
using namespace units::length;
using namespace units::time;
using namespace units::velocity;
using namespace units::literals;
using namespace units::angle;
using namespace units::constants;
using namespace units::dimensionless;
using namespace units::angular_velocity;

/*!
 * \brief A tool for moving a tank robot to a specified position in space
 *
 * This class requires that we have a mechanism for obtaining the robot's pose,
 * with, e.g. a Vicon system (see Vicon::UDPClient). The algorithm used is drawn
 * from: https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
 */
class RobotPositioner
{
private:
    // Robot variables
    Pose2<millimeter_t, degree_t> m_RobotPose;
    Pose2<millimeter_t, degree_t> m_GoalPose;
    millimeter_t m_DistanceFromGoal; // Euclidean distance from goal coordinate
    degree_t m_BearingFromGoal;      // bearing (angle) from goal coordinate

    // user variables
    const millimeter_t m_StoppingDistance;          // if the robot's distance from goal < stopping dist, robot stops
    const degree_t m_AllowedHeadingError;          // the amount of error allowed in the final heading
    const meters_per_second_t m_MaxVelocity;        // max velocity
    const radians_per_second_t m_MaxTurnSpeed; // max turning velocity
    const double m_K1;                               // curveness of the path to the goal
    const double m_K2;                               // speed of turning on the curves
    const double m_Alpha;                            // causes more sharply peaked curves
    const double m_Beta;                             // causes to drop velocity if 'k'(curveness) increases

    template<typename AngleUnit>
    static AngleUnit angleWrapAround(AngleUnit angle) {

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
    void updateRangeAndBearing() {

        const millimeter_t delta_x = m_GoalPose.x - m_RobotPose.x;
        const millimeter_t delta_y = m_GoalPose.y - m_RobotPose.y;

        // calculate distance
        m_DistanceFromGoal =  units::math::hypot(delta_x, delta_y);

        // calculate bearing
        m_BearingFromGoal = units::math::atan2(delta_y,delta_x) - m_RobotPose.angle;

        // changing from <0,360> to <-180, 180>
        m_BearingFromGoal = angleWrapAround(m_BearingFromGoal);
        m_RobotPose.angle = angleWrapAround(m_RobotPose.angle);
    }

//-----------------PUBLIC API---------------------------------------------------------------------
public:

    RobotPositioner(

        millimeter_t stopping_distance,                               // if the robot's distance from goal < stopping dist, robot stops
        degree_t allowed_heading_error,                               // the amount of error allowed in the final heading
        double k1,                                                    // curveness of the path to the goal
        double k2,                                                    // speed of turning on the curves
        double alpha,                                                 // causes more sharply peaked curves
        double beta,                                                  // causes to drop velocity if 'k'(curveness) increases
        meters_per_second_t max_velocity,                             // max velocity
        radians_per_second_t max_turning_velocity
        ) : m_StoppingDistance(stopping_distance),
            m_AllowedHeadingError(allowed_heading_error),
            m_MaxVelocity(max_velocity),
            m_MaxTurnSpeed(max_turning_velocity),
            m_K1(k1),
            m_K2(k2),
            m_Alpha(alpha),
            m_Beta(beta)
    {  }

    //! sets the goal pose (x, y, angle)
    void setGoalPose(const Pose2<millimeter_t, degree_t> &pose)
    {
        m_GoalPose = pose;
        updateRangeAndBearing();
    }

    //! updates the agent's current pose
    void setPose(const Pose2<millimeter_t, degree_t> &pose)
    {
        m_RobotPose = pose;

        // Recompute heading and distance from goal
        updateRangeAndBearing();
    }

    //! updates the velocities in order to get to a goal location. This function can be used
    //! without a robot interface, where only velocities are calculated but no robot actions
    //! will be executed.
    void updateVelocities(
            meters_per_second_t &v,  // velocity to update
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
        if (m_DistanceFromGoal == 0_mm) {
            v = 0_mps;
            omega = (m_BearingFromGoal < 0_deg) ? -m_MaxTurnSpeed : m_MaxTurnSpeed;
            return;
        }

        // orientation of Target with respect to the line of sight from the observer to the target
        radian_t theta = m_RobotPose.angle + m_BearingFromGoal - m_GoalPose.angle;
        theta = angleWrapAround(theta);

        const radian_t delta = units::math::atan(scalar_t((-m_K1 * theta).value()));
        const radian_t part1 = m_K2 * (static_cast<radian_t>(m_BearingFromGoal) - delta);
        const radian_t part2 = (1_rad + radian_t{ (m_K1 / (1 +
                                                           units::math::pow<2>(m_K1 * theta).value())) }) *
                               units::math::sin(m_BearingFromGoal);

        const auto k = -(part1 + part2) / m_DistanceFromGoal; // in rad/mm

        v = m_MaxVelocity / scalar_t((1 + m_Beta * pow(std::abs(k.value()), m_Alpha)));
        omega = -k * v;

        /*
         * We want to cap the turning speed at m_MaxTurnSpeed, but we also need
         * to recalculate v so that the centre of the robot's turning circle
         * stays the same.
         */
        if (units::math::abs(omega) > m_MaxTurnSpeed) {
            const meter_t r{ (v / omega).value() };
            omega = (omega < 0_rad_per_s) ? -m_MaxTurnSpeed : m_MaxTurnSpeed;
            v = meters_per_second_t{ (omega * r).value() };
        }
    }

    //! This function will update the motors so it drives towards a previously set goal location
    void updateMotors(BoBRobotics::Robots::Tank &bot,
                      const Pose2<millimeter_t, degree_t> &pose)
    {
        // here we are solving simultaneous equations where we would like to
        // recover Vl (left wheel velocity) and Vr (right wheel velocity)
        //                                       v = wheel_radius * (Vl+Vr)/2
        //                                       w = wheel_radius * (Vr-Vl)/axis_length
        setPose(pose);
        if (didReachGoal()) {
            bot.stopMoving();
            return;
        }

        meters_per_second_t v;
        radians_per_second_t omega;
        updateVelocities(v, omega);

        const millimeter_t robot_wheel_radius = bot.getRobotWheelRadius();
        const millimeter_t robot_axis_length  = bot.getRobotAxisLength();

        const double a = (robot_wheel_radius/2).value();
        const double b = (robot_wheel_radius/robot_axis_length).value();

        const double c = v.value();
        const double d = static_cast<units::angular_velocity::radians_per_second_t>(omega).value();

        // determinant
        const double det = 2*(-a*b);

        const double Vl = ((-c*b - a*d)/det);
        const double Vr = (( a*d - c*b)/det);

        // drive robot
        std::cout << "Motor: " << Vl << ", " << Vr << std::endl;
        const auto cap = [](const double val) {
            return std::min(1.0, std::max(-1.0, val));
        };
        bot.tank(cap(Vl), cap(Vr));
    }

    //! returns true if the robot reached the goal position
    bool didReachGoal()
    {
        return m_DistanceFromGoal < m_StoppingDistance &&
                    units::math::abs(m_RobotPose.angle - m_GoalPose.angle) < m_AllowedHeadingError;
    }

}; // RobotPositioner
} // Robots
} // BoBRobotics
