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

template<typename LengthUnit = units::length::millimeter_t,
         typename AngleUnit = units::angle::degree_t>
class GazeboTank
  : public Tank
{
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    GazeboTank(const meters_per_second_t maximumSpeed, gazebo::transport::NodePtr node)
      : m_MaximumSpeed(maximumSpeed)
    {
        // Publish to the  differential_drive_robot topic
        pub =node->Advertise<gazebo::msgs::Vector3d>("~/differential_drive_robot/vel_cmd");;
        // Wait for a subscriber to connect to this publisher
        pub->WaitForConnection();
    }

    const auto &getWheelSpeeds(){
        return m_WheelSpeeds;
    }
    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override
    {
        return m_MaximumSpeed;
    }

    virtual void tank(float left, float right) override
    {
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);
        m_WheelSpeeds.first = left * m_MaximumSpeed;
        m_WheelSpeeds.second = right * m_MaximumSpeed;

          // Set the velocity in the x-component
        gazebo::msgs::Set(&msg, ignition::math::Vector3d((float)m_WheelSpeeds.first, (float)m_WheelSpeeds.second, 0));
        // Send the message
        pub->Publish(msg);
        // std::cout<< wheelSpeeds.first << "," << wheelSpeeds.second << std::endl; 
        LOG_DEBUG << m_WheelSpeeds.first << "," << m_WheelSpeeds.second;       

    }

private:

    const meters_per_second_t m_MaximumSpeed;
    std::pair <meters_per_second_t, meters_per_second_t> m_WheelSpeeds;
    gazebo::msgs::Vector3d msg;
    gazebo::transport::PublisherPtr pub;
}; // GazeboTank
} // Robots
} // BoBRobotics
