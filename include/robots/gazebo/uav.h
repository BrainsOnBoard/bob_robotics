#pragma once

// BoB robotics includes
#include "robots/uav.h"

// Gazebo includes
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {
class UAV
  : public Robots::UAV
{
public:
    UAV(gazebo::transport::NodePtr node)
    {
        // Publish to the  gazebo_quadcopter topic
        m_Pub =node->Advertise<gazebo::msgs::Quaternion>("~/gazebo_quadcopter/motors_cmd");;
        // Wait for a subscriber to connect to this publisher
        m_Pub->WaitForConnection();
    }
    virtual void takeOff() override;
    virtual void land() override;
    virtual void setPitch(float pitch) override;
    virtual void setRoll(float right) override;
    virtual void setVerticalSpeed(float up) override;
    virtual void setYawSpeed(float right) override;
    virtual void sendCommand();

private:
    gazebo::transport::PublisherPtr m_Pub;
    float m_Thrust, m_Roll, m_Pitch, m_Yaw;
    std::atomic_bool m_Armed;
    std::mutex m_Mutex;
}; // UAV
} // Gazebo
} // Robots
} // BoBRobotics
