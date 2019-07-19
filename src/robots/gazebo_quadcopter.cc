// BoB robotics includes
#include "robots/gazebo_quadcopter.h"

namespace BoBRobotics {
namespace Robots {

void GazeboQuadCopter::takeOff()
{
    m_Thrust = 0.6;
    motorMixing();
}

void GazeboQuadCopter::land()
{
    m_Thrust = 0.5;
    motorMixing();
}
void GazeboQuadCopter::setPitch(float pitch)
{
    m_Pitch = pitch*0.01;
    motorMixing();
}
void GazeboQuadCopter::setRoll(float right)
{
    m_Roll = right*0.01;
    motorMixing();
}
void GazeboQuadCopter::setVerticalSpeed(float up)
{
    m_Thrust = up*0.5 + 0.6;
    motorMixing();
}
void GazeboQuadCopter::setYawSpeed(float right)
{
    m_Yaw = right*0.01;
    motorMixing();
}
void GazeboQuadCopter::motorMixing()
{
    float m_fr, m_fl, m_br, m_bl;
    m_fr = m_Thrust + m_Yaw - m_Pitch - m_Roll;
    m_fl = m_Thrust - m_Yaw - m_Pitch + m_Roll;
    m_br = m_Thrust - m_Yaw + m_Pitch - m_Roll;
    m_bl = m_Thrust + m_Yaw + m_Pitch + m_Roll;
    std::cout << m_fl << "\t\t"<< m_fr << std::endl << "\tX\n" <<  m_bl<< "\t\t" << m_br << std::endl;
    gazebo::msgs::Set(&msg, ignition::math::Quaterniond(m_fr, m_bl, m_fl, m_br));
    // Send the message
    pub->Publish(msg);
}

} // Robots
} // BoBRobotics
