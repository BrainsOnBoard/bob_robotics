// BoB robotics includes
#include "robots/gazebo_uav.h"
#include "third_party/units.h"
using namespace std::literals;
using namespace units::literals;
namespace BoBRobotics {
namespace Robots {

void GazeboQuadCopter::takeOff()
{
    m_Armed=true;
    m_Thrust = 1;
    sendCommand();
    std::this_thread::sleep_for(500ms);
    m_Thrust = 0.5;
    sendCommand();
}

void GazeboQuadCopter::land()
{
    m_Armed=false;
    m_Thrust = 0.4;
    sendCommand();
}
void GazeboQuadCopter::setPitch(float pitch)
{
    if(!m_Armed){
        return;
    }
    m_Pitch = pitch;
    sendCommand();
}
void GazeboQuadCopter::setRoll(float roll)
{
    if(!m_Armed){
        return;
    }
    m_Roll = roll;
    sendCommand();
}
void GazeboQuadCopter::setVerticalSpeed(float up)
{
    if(!m_Armed){
        return;
    }
    m_Thrust = up*0.5 + 0.5;
    sendCommand();
}
void GazeboQuadCopter::setYawSpeed(float yaw)
{
    if(!m_Armed){
        return;
    }
    m_Yaw = yaw;
    sendCommand();
}
void GazeboQuadCopter::sendCommand()
{
    gazebo::msgs::Quaternion msg;
    gazebo::msgs::Set(&msg, ignition::math::Quaterniond(m_Thrust, m_Roll, m_Pitch, m_Yaw));
    m_Pub->Publish(msg);
}

} // Robots
} // BoBRobotics
