// BoB robotics includes
#include "common/macros.h"
#include "robots/simulated_ackermann.h"

using namespace units::angle;
using namespace units::length;
using namespace units::math;
using namespace units::time;
using namespace units::velocity;

namespace BoBRobotics {
namespace Robots {

SimulatedAckermann::SimulatedAckermann(const meters_per_second_t maximumSpeed,
                                       const meter_t axisDist,
                                       const meter_t carHeight,
                                       const degree_t maxTurn)
  : m_MaximumSpeed(maximumSpeed)
  , m_MaximumTurn(maxTurn)
  , m_distanceBetweenAxis(axisDist)
{
    m_Pose.z() = carHeight;
}

meter_t
SimulatedAckermann::getDistanceBetweenAxis() const
{
    return m_distanceBetweenAxis;
}

const Pose3<meter_t, degree_t> &
SimulatedAckermann::getPose()
{
    updatePose();
    return m_Pose;
}

meters_per_second_t
SimulatedAckermann::getAbsoluteMaximumSpeed() const
{
    return m_MaximumSpeed;
}

degree_t
SimulatedAckermann::getMaximumTurn() const
{
    return m_MaximumTurn;
}

void
SimulatedAckermann::setPose(const Pose3<meter_t, degree_t> &pose)
{
    m_MoveStopwatch.start();
    m_Pose = pose;
}

void
SimulatedAckermann::moveForward(float speed)
{
    // Check value is in range
    BOB_ASSERT(speed >= -1.f && speed <= 1.f);

    updatePose();
    m_currentVelocity = speed * m_MaximumSpeed;
}

void
SimulatedAckermann::steer(float value)
{
    // Check value is in range
    BOB_ASSERT(value >= -1.f && value <= 1.f);

    updatePose();
    m_steeringWheelAngle = value * m_MaximumTurn;
}

void
SimulatedAckermann::steer(degree_t steeringAngle)
{
    BOB_ASSERT(abs(steeringAngle) <= m_MaximumTurn);
    updatePose();
    m_steeringWheelAngle = steeringAngle;
}

void
SimulatedAckermann::stopMoving()
{
    move(0_mps, 0_deg);
}

void
SimulatedAckermann::move(float velocity, degree_t steeringAngle)
{
    // Check values are in range
    BOB_ASSERT(velocity >= -1.f && velocity <= 1.f);
    BOB_ASSERT(abs(steeringAngle) <= m_MaximumTurn);

    updatePose();
    m_currentVelocity = velocity * m_MaximumSpeed;
    m_steeringWheelAngle = steeringAngle;
}

void
SimulatedAckermann::move(meters_per_second_t velocity, degree_t steeringAngle)
{
    // Check values are in range
    BOB_ASSERT(abs(velocity) <= m_MaximumSpeed);
    BOB_ASSERT(abs(steeringAngle) <= m_MaximumTurn);

    updatePose();
    m_currentVelocity = velocity;
    m_steeringWheelAngle = steeringAngle;
}

void
SimulatedAckermann::updatePose()
{
    const second_t elapsed = m_MoveStopwatch.lap();

    m_Pose.x() += m_currentVelocity * cos(m_Pose.yaw()) * elapsed;
    m_Pose.y() += m_currentVelocity * sin(m_Pose.yaw()) * elapsed;

    double delta_angle = m_currentVelocity * elapsed / m_distanceBetweenAxis * tan(m_steeringWheelAngle);
    m_Pose.yaw() += radian_t(delta_angle);
}
} // Robots
} // BoBRobotics
