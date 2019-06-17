#pragma once

// BoB robotics includes
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

class SimulatedAckermann
{
    using meter_t = units::length::meter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using degree_t = units::angle::degree_t;
    using second_t = units::time::second_t;

public:
    SimulatedAckermann(const meters_per_second_t maximumSpeed,
                       const meter_t axisDist,
                       const meter_t carHeight = 0_m,
                       const degree_t maxTurn = 45_deg);

#ifdef USE_BOB_HID
    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f);
#endif

    meter_t getDistanceBetweenAxis() const;

    template<typename LengthUnit = meter_t>
    Vector3<LengthUnit> getPosition()
    {
        updatePose();
        return { m_Pose.x(), m_Pose.y(), m_Pose.z() };
    }

    template<typename AngleUnit = degree_t>
    std::array<AngleUnit, 3> getAttitude()
    {
        updatePose();
        return { m_Pose.yaw(), 0_rad, 0_rad };
    }

    const Pose3<meter_t, degree_t> &getPose();
    meters_per_second_t getAbsoluteMaximumSpeed() const;
    degree_t getMaximumTurn() const;
    void setPose(const Pose3<meter_t, degree_t> &pose);
    void moveForward(float speed);
    void steer(float value);
    void steer(degree_t steeringAngle);
    void stopMoving() noexcept;

    //! sets the robot velocity and steering to move
    void move(meters_per_second_t velocity, degree_t steeringAngle);

private:
    Pose3<meter_t, degree_t> m_Pose;
    Stopwatch m_MoveStopwatch;
    const meters_per_second_t m_MaximumSpeed;
    const degree_t m_MaximumTurn;
    const meter_t m_distanceBetweenAxis; // distance between front and rear axis
    degree_t m_steeringWheelAngle{};     // steering wheel angle
    meters_per_second_t m_currentVelocity{};

    void updatePose();
}; // SimulatedAckermann
} // Robots
} // BoBRobotics

