// BoB robotics includes

#include "common/macros.h"
#include "robots/gazebo/rc_car.h"
#include "third_party/units.h"

using namespace units::angular_velocity;
using namespace units::velocity;

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

RCCar::RCCar(const radians_per_second_t maximumSpeed,
            gazebo::transport::NodePtr node)
    : m_MaximumSpeed(maximumSpeed)
{
    // Publish to the  ackermann_drive_robot topic
    pub = node->Advertise<gazebo::msgs::Vector2d>("~/cart_front_steer_pancam/vel_cmd");

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();
}

void RCCar::start(gazebo::transport::NodePtr node)
{

    sub = node->Subscribe("~/cart_front_steer_pancam/pose", &RCCar::poseUpdate,this);
}

void RCCar::poseUpdate(ConstPosePtr &_msg)
{

    std::cout << _msg->position().x() << " " << _msg->position().y() << " " << _msg->position().z() << std::endl;
}



meters_per_second_t RCCar::getAbsoluteMaximumSpeed()
{
    return meters_per_second_t{ 0.3 * m_MaximumSpeed.value() }; //radius of the wheel is 0.3m. v = r × ω
}

void RCCar::move(float speed, float steering)
{
    //BOB_ASSERT(speed >= -1.5);
    //BOB_ASSERT(right >= -35.f && right <= 35.f);

    // Set the velocity in the x-component
    gazebo::msgs::Set(&msg, ignition::math::Vector2d(speed * m_MaximumSpeed.value(), steering ));

    // Send the message
    pub->Publish(msg);
}

} // Gazebo
} // Robots
} // BoBRobotics
