#pragma once

// BoB robotics includes
#include "robots/uav.h"

// Gazebo includes
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace BoBRobotics {
namespace Robots {
class GazeboQuadCopter
  : public UAV
{
public:
    GazeboQuadCopter(gazebo::transport::NodePtr node)
    {
        // Publish to the  gazebo_quadcopter topic
        pub =node->Advertise<gazebo::msgs::Quaternion>("~/gazebo_quadcopter/motors_cmd");;
        // Wait for a subscriber to connect to this publisher
        pub->WaitForConnection();
    }
    virtual void takeOff() override;
    virtual void land() override;
    virtual void setPitch(float pitch) override;
    virtual void setRoll(float right) override;
    virtual void setVerticalSpeed(float up) override;
    virtual void setYawSpeed(float right) override;
    virtual void motorMixing();

private:
    gazebo::msgs::Quaternion msg;
    gazebo::transport::PublisherPtr pub;
    float m_Thrust, m_Roll, m_Pitch, m_Yaw;
};
} // Robots
} // BoBRobotics
