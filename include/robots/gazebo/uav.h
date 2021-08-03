#pragma once

// BoB robotics includes
#include "robots/uav/uav_base.h"

// Gazebo includes
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {
class UAV
  : public Robots::UAV::UAVBase<UAV>
{
public:
    UAV(gazebo::transport::NodePtr node)
    {
        // Publish to the  gazebo_quadcopter topic
        m_Pub =node->Advertise<gazebo::msgs::Quaternion>("~/gazebo_quadcopter/motors_cmd");;
        // Wait for a subscriber to connect to this publisher
        m_Pub->WaitForConnection();
    }
    void takeOff();
    void land();
    void setPitch(float pitch);
    void setRoll(float roll);
    void setVerticalSpeed(float up);
    void setYawSpeed(float right);
    void sendCommand();

private:
    gazebo::transport::PublisherPtr m_Pub;
    float m_Thrust{}, m_Roll{}, m_Pitch{}, m_Yaw{};
    std::atomic_bool m_Armed;
    std::mutex m_Mutex;
}; // UAV
} // Gazebo
} // Robots
} // BoBRobotics
