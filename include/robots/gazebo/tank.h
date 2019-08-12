#pragma once

// BoB robotics includes
#include "robots/tank.h"

// Third-party includes
#include "third_party/units.h"

// Gazebo includes
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

class Tank : public Robots::Tank
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    Tank(const radians_per_second_t maximumSpeed,
         gazebo::transport::NodePtr node);

    // Public virtual methods
    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override;
    virtual void tank(float left, float right) override;

private:
    const radians_per_second_t m_MaximumSpeed;
    gazebo::msgs::Vector2d msg;
    gazebo::transport::PublisherPtr pub;

}; // Tank
} // Gazebo
} // Robots
} // BoBRobotics
