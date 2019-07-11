// BoB robotics includes
#include "robots/gazebo_quadcopter.h"

namespace BoBRobotics {
namespace Robots {

void GazeboQuadCopter::takeOff()
{
    //takeoff
}

void GazeboQuadCopter::land()
{
    //land
}
void GazeboQuadCopter::setPitch(float pitch)
{
    m_Pitch = pitch;
    motorMixing();
}
void GazeboQuadCopter::setRoll(float right)
{
    m_Roll = right;
    motorMixing();
}
void GazeboQuadCopter::setVerticalSpeed(float up)
{
    m_Thrust = up;
    motorMixing();
}
void GazeboQuadCopter::setYawSpeed(float right)
{
    m_Yaw = right;
    motorMixing();
}
void GazeboQuadCopter::motorMixing()
{
    float m_fr, m_fl, m_br, m_bl;
    m_fr = m_Thrust + m_Yaw + m_Pitch + m_Roll;
    m_fl = m_Thrust - m_Yaw + m_Pitch - m_Roll;
    m_br = m_Thrust - m_Yaw - m_Pitch + m_Roll;
    m_bl = m_Thrust + m_Yaw - m_Pitch - m_Roll;
    // std::cout << m_fr<< m_fl<< m_br<< m_bl << std::endl;
    gazebo::msgs::Set(&msg, ignition::math::Quaterniond(m_fr, m_bl, m_fl, m_br));
    // Send the message
    pub->Publish(msg);
}

} // Robots
} // BoBRobotics
