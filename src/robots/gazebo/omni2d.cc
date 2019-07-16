// BoB robotics includes
#include "robots/gazebo/omni2d.h"
#include "common/circstat.h"
#include "common/macros.h"

using namespace units::literals;
using namespace units::angular_velocity;
using namespace units::velocity;

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {
Omni2D::Omni2D(gazebo::transport::Node &node,
               const meters_per_second_t maximumSpeed = 1_mps)
  : m_Publisher(node.Advertise<gazebo::msgs::Vector3d>(TopicName))
  , m_MaximumSpeed(getAngularVelocity(maximumSpeed, WheelRadius))
{
    // Wait for a subscriber to connect to this publisher
    m_Publisher->WaitForConnection();
}

meters_per_second_t
Omni2D::getAbsoluteMaximumSpeed() const
{
    return getVelocity(m_MaximumSpeed, WheelRadius);
}

void
Omni2D::omni2D(float forward, float sideways, float turn)
{
    BOB_ASSERT(forward >= -1.f && forward <= 1.f);
    BOB_ASSERT(sideways >= -1.f && sideways <= 1.f);
    BOB_ASSERT(turn >= -1.f && turn <= 1.f);

    const double max = m_MaximumSpeed.value();
    gazebo::msgs::Set(&m_Message, ignition::math::Vector3d(forward * max, sideways * max, turn * max));

    // Send the message
    m_Publisher->Publish(m_Message);
}

} // Gazebo
} // Robots
} // BoBRobotics
