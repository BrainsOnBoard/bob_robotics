#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/pose.h"
#include "common/stopwatch.h"
#ifdef USE_BOB_HID
#include "hid/joystick.h"
#endif

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
    SimulatedAckermannCar(const meters_per_second_t maximumSpeed,
                          const LengthUnit axisDist,
                          const LengthUnit carHeight = meters_per_second_t{ 0 },
                          const AngleUnit maxTurn = degree_t{ 45 })
      : m_MaximumSpeed(maximumSpeed)
      , m_MaximumTurn(maxTurn)
      , m_distanceBetweenAxis(axisDist)
    {
        m_Pose.z() = carHeight;
    }

#ifdef USE_BOB_HID
    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f)
    {
        joystick.addHandler([this](HID::JAxis axis, float value)
            {
                if (axis == HID::JAxis::LeftStickVertical) {
                    moveForward(-value);
                    return true;
                } else if (axis == HID::JAxis::RightStickHorizontal) {
                    steer(value);
                    return true;
                } else {
                    return false;
                }
            });
    }
#endif

    millimeter_t getDistanceBetweenAxis() const
    {
        return m_distanceBetweenAxis;
    }

    template<typename ReturnLengthUnit = LengthUnit>
    Vector3<ReturnLengthUnit> getPosition()
    {
        updatePose();
        return { m_Pose.x(), m_Pose.y(), m_Pose.z() };
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

    meters_per_second_t getAbsoluteMaximumSpeed() const
    {
        return m_MaximumSpeed;
    }

    void setPose(const Pose2<LengthUnit, AngleUnit> &pose)
    {
        m_MoveStopwatch.start();
        m_Pose = pose;
    }

    void moveForward(float speed)
    {
        // Check value is in range
        BOB_ASSERT(speed >= -1.f && speed <= 1.f);

        updatePose();
        m_currentVelocity = speed * m_MaximumSpeed;
    }

    void steer(float value)
    {
        // Check value is in range
        BOB_ASSERT(value >= -1.f && value <= 1.f);

        updatePose();
        m_steeringWheelAngle = -value * m_MaximumTurn;
    }

    void stopMoving() noexcept
    {
        move(meters_per_second_t{ 0 }, AngleUnit{ 0 });
    }

    //! sets the robot velocity and steering to move
    void move(meters_per_second_t velocity, AngleUnit steeringAngle)
    {
        // Check values are in range
        using namespace units::math;
        BOB_ASSERT(abs(velocity) <= m_MaximumSpeed);
        BOB_ASSERT(abs(steeringAngle) <= m_MaximumTurn);

        updatePose();
        m_currentVelocity = velocity;
        m_steeringWheelAngle = -steeringAngle;
    }

private:
    Pose3<LengthUnit, AngleUnit> m_Pose;
    Stopwatch m_MoveStopwatch;
    const meters_per_second_t m_MaximumSpeed;
    const AngleUnit m_MaximumTurn;
    const LengthUnit m_distanceBetweenAxis; // distance between front and rear axis
    AngleUnit m_steeringWheelAngle{};       // steering wheel angle
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

