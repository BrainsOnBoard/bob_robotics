// BoB robotics includes
#include "common/macros.h"
#include "robots/gazebo/tank.h"

using namespace units::angular_velocity;
using namespace units::velocity;

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

Tank::Tank(const radians_per_second_t maximumSpeed,
            gazebo::transport::NodePtr node)
    : m_MaximumSpeed(maximumSpeed)
{
    // Publish to the  differential_drive_robot topic
    pub = node->Advertise<gazebo::msgs::Vector2d>("~/differential_drive_robot/vel_cmd");

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();
}

meters_per_second_t Tank::getAbsoluteMaximumSpeed() const
{
    return meters_per_second_t{ 0.3 * m_MaximumSpeed.value() }; //radius of the wheel is 0.3m. v = r × ω
}

void Tank::tank(float left, float right)
{
    BOB_ASSERT(left >= -1.f && left <= 1.f);
    BOB_ASSERT(right >= -1.f && right <= 1.f);

    // Set the velocity in the x-component
    gazebo::msgs::Set(&msg, ignition::math::Vector2d(left * m_MaximumSpeed.value(), right * m_MaximumSpeed.value()));

    // Send the message
    pub->Publish(msg);
}

} // Gazebo
} // Robots
} // BoBRobotics
