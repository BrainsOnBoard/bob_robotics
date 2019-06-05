#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "../common/stopwatch.h"
#include "tank.h"

// Third-party includes
#include "../third_party/units.h"
#include <utility> 

// Gazebo includes
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;


class GazeboTank
  : public Tank
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;
public:
    GazeboTank(const radians_per_second_t maximumSpeed, gazebo::transport::NodePtr node)
      : m_MaximumSpeed(maximumSpeed)
    {
        // Publish to the  differential_drive_robot topic
        pub =node->Advertise<gazebo::msgs::Vector3d>("~/differential_drive_robot/vel_cmd");;
        // Wait for a subscriber to connect to this publisher
        pub->WaitForConnection();
    }

    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override
    {
        return meters_per_second_t{ 0.3 * m_MaximumSpeed.value() }; //radius of the wheel is 0.3m. v = r × ω
    }

    virtual void tank(float left, float right) override
    {
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);
          // Set the velocity in the x-component
        gazebo::msgs::Set(&msg, ignition::math::Vector3d(left * m_MaximumSpeed.value(), right * m_MaximumSpeed.value(), 0));
        // Send the message
        pub->Publish(msg);
    }

private:
    const radians_per_second_t m_MaximumSpeed;
    gazebo::msgs::Vector3d msg;
    gazebo::transport::PublisherPtr pub;
}; // GazeboTank
} // Robots
} // BoBRobotics
