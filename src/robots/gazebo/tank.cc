// BoB robotics includes
#include "common/circstat.h"
#include "common/macros.h"
#include "robots/gazebo/camera.h"
#include "robots/gazebo/tank.h"

using namespace units::angular_velocity;
using namespace units::velocity;

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

Tank::Tank(const meters_per_second_t maximumSpeed,
           gazebo::transport::NodePtr node)
  : m_MaximumSpeed(getAngularVelocity(maximumSpeed, 0.3_m))
  , m_Node(node)
  , m_Pub(node->Advertise<gazebo::msgs::Vector2d>("~/differential_drive_robot/vel_cmd"))
{
    // Wait for a subscriber to connect to this publisher
    m_Pub->WaitForConnection();
}

meters_per_second_t
Tank::getAbsoluteMaximumSpeed() const
{
    return getVelocity(m_MaximumSpeed, 0.3_m); //radius of the wheel is 0.3m
}

void
Tank::tank(float left, float right)
{
    BOB_ASSERT(left >= -1.f && left <= 1.f);
    BOB_ASSERT(right >= -1.f && right <= 1.f);

    // Set the velocity in the x-component
    ignition::math::Vector2d vec{ left * m_MaximumSpeed.value(),
                                  right * m_MaximumSpeed.value() };
    gazebo::msgs::Set(&m_Msg, vec);

    // Send the message
    m_Pub->Publish(m_Msg);
}

std::unique_ptr<Video::Input>
Tank::getCamera()
{
    return std::make_unique<Camera>(m_Node);
}

} // Gazebo
} // Robots
} // BoBRobotics
