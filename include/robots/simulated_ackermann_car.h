#pragma once

// BoB robotics includes
#include "common/pose.h"
#include "common/stopwatch.h"


// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

template<typename LengthUnit = units::length::millimeter_t,
         typename AngleUnit = units::angle::degree_t>
class SimulatedAckermannCar
{
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using degree_t = units::angle::degree_t;
    using second_t = units::time::second_t;

public:
    SimulatedAckermannCar(const meters_per_second_t maximumSpeed, const millimeter_t axis_dist)
      : m_MaximumSpeed(maximumSpeed)
      , m_distanceBetweenAxis(axis_dist)
    {}

    millimeter_t getDistanceBetweenAxis()
    {
        return m_distanceBetweenAxis;
    }

    template<typename ReturnLengthUnit = LengthUnit>
    Vector3<ReturnLengthUnit> getPosition()
    {
        updatePose();
        return { m_Pose.x(), m_Pose.y(), 0_m };
    }

    template<typename ReturnAngleUnit = AngleUnit>
    std::array<ReturnAngleUnit, 3> getAttitude()
    {
        updatePose();
        return { m_Pose.yaw(), 0_rad, 0_rad };
    }

    const auto &getPose()
    {
        updatePose();
        return m_Pose;
    }

    meters_per_second_t getAbsoluteMaximumSpeed() 
    {
        return m_MaximumSpeed;
    }

    void setPose(const Pose2<LengthUnit, AngleUnit> &pose)
    {
        m_MoveStopwatch.start();
        m_Pose = pose;
    }

    //! sets the robot velocity and steering to move
    void move(meters_per_second_t velocity, degree_t steeringAngle) {
        updatePose();
        m_currentVelocity = velocity;
        m_steeringWheelAngle = steeringAngle;
    }

    


private:
    Pose2<LengthUnit, AngleUnit> m_Pose;
    Stopwatch m_MoveStopwatch;
    const meters_per_second_t m_MaximumSpeed;
    const millimeter_t m_distanceBetweenAxis;  // distance between front and rear axis
    degree_t m_steeringWheelAngle{};           // steering wheel angle
    meters_per_second_t m_currentVelocity{};

    void updatePose()
    {
        using namespace units::angle;
        using namespace units::length;
        using namespace units::math;
        using namespace units::time;

        const second_t elapsed = m_MoveStopwatch.lap();
        
        m_Pose.x() += m_currentVelocity * cos(m_Pose.yaw()) * elapsed;
        m_Pose.y() += m_currentVelocity * sin(m_Pose.yaw()) * elapsed;
        
        double delta_angle = m_currentVelocity * elapsed / m_distanceBetweenAxis * tan(m_steeringWheelAngle);
        m_Pose.yaw() += radian_t(delta_angle);

    }
}; // SimulatedTank
} // Robots
} // BoBRobotics

