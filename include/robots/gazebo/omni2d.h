#pragma once

// Third-party includes
#include "third_party/units.h"

// Gazebo
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {
using namespace units::literals;

class Omni2D
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    static constexpr const char *TopicName = "~/robot_mobile_wheel_3_omni_open_base/vel_cmd";
    static constexpr units::length::meter_t WheelRadius = 1.905_cm;

    Omni2D(gazebo::transport::Node &node,
           const meters_per_second_t maximumSpeed);

    meters_per_second_t getAbsoluteMaximumSpeed() const;
    void drive(float left, float right, float back);

private:
    gazebo::transport::PublisherPtr m_Publisher;
    const radians_per_second_t m_MaximumSpeed;
    gazebo::msgs::Vector3d m_Message;
}; // Omni2D
} // Gazebo
} // Robots
} // BoBRobotics
