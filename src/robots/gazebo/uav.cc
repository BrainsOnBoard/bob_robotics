// BoB robotics includes
#include "robots/gazebo/uav.h"

// Third-party includes
#include "third_party/units.h"

using namespace std::literals;
using namespace units::literals;

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

void UAV::takeOff()
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_Armed=true;
    m_Thrust = 1;
    sendCommand();
    std::this_thread::sleep_for(500ms);
    m_Thrust = 0.5;
    sendCommand();
}

void UAV::land()
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_Armed=false;
    m_Thrust = 0.4;
    sendCommand();
}
void UAV::setPitch(float pitch)
{
    if(!m_Armed){
        return;
    }
    m_Pitch = pitch;
    sendCommand();
}
void UAV::setRoll(float roll)
{
    if(!m_Armed){
        return;
    }
    m_Roll = roll;
    sendCommand();
}
void UAV::setVerticalSpeed(float up)
{
    if(!m_Armed){
        return;
    }
    m_Thrust = up*0.5 + 0.5;
    sendCommand();
}
void UAV::setYawSpeed(float yaw)
{
    if(!m_Armed){
        return;
    }
    m_Yaw = yaw;
    sendCommand();
}
void UAV::sendCommand()
{
    gazebo::msgs::Quaternion msg;
    gazebo::msgs::Set(&msg, ignition::math::Quaterniond(m_Thrust, m_Roll, m_Pitch, m_Yaw));
    m_Pub->Publish(msg);
}

} // Gazebo
} // Robots
} // BoBRobotics
