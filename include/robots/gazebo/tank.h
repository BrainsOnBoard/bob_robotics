#pragma once

// BoB robotics includes
#include "common/pose.h"
#include "common/stopwatch.h"
#include "robots/tank.h"

// Third-party includes
#include "../third_party/units.h"

// Gazebo includes
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

// Standard c++ includes
#include <utility>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {
using namespace units::literals;

class Tank : public Robots::Tank
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    Tank(const radians_per_second_t maximumSpeed, gazebo::transport::NodePtr node)
      : m_MaximumSpeed(maximumSpeed)
    {
        // Publish to the  differential_drive_robot topic
        pub = node->Advertise<gazebo::msgs::Vector2d>("~/differential_drive_robot/vel_cmd");

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
        gazebo::msgs::Set(&msg, ignition::math::Vector2d(left * m_MaximumSpeed.value(), right * m_MaximumSpeed.value()));

        // Send the message
        pub->Publish(msg);
    }

private:
    const radians_per_second_t m_MaximumSpeed;
    gazebo::msgs::Vector2d msg;
    gazebo::transport::PublisherPtr pub;
};// GazeboTank
} // Gazebo
} // Robots
} // BoBRobotics
