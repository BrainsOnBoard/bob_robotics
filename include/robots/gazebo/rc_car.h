#pragma once

// Third-party includes
#include "third_party/units.h"

// Gazebo includes
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

class RCCar
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    RCCar(const radians_per_second_t maximumSpeed,
         gazebo::transport::NodePtr node);

    // Public methods
    meters_per_second_t getAbsoluteMaximumSpeed();
    void move(float speed, float steeringAngle) ;
    void poseUpdate(ConstPosePtr &_msg);
    void start(gazebo::transport::NodePtr node);

private:
    const radians_per_second_t m_MaximumSpeed;
    gazebo::msgs::Vector2d msg;
    gazebo::transport::PublisherPtr pub; // the node to communicate with the simulation
    gazebo::transport::SubscriberPtr sub; // pose

}; // RCCar
} // Gazebo
} // Robots
} // BoBRobotics
