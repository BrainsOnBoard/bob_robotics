#pragma once

// BoB robotics includes
#include "robots/tank/tank_base.h"
#include "robots/gazebo/node.h"

// Third-party includes
#include "third_party/units.h"

// Gazebo includes
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

using namespace units::literals;

class Tank : public Robots::TankBase
{
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    Tank(const meters_per_second_t maximumSpeed = 1_mps,
         gazebo::transport::NodePtr node = getNode());

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
