// BoB robotics includes
#include "robots/gazebo_quadcopter.h"

namespace BoBRobotics {
namespace Robots {

void GazeboQuadCopter::takeOff()
{
    m_Thrust = 1;
    sendCommand();
}

void GazeboQuadCopter::land()
{
    m_Thrust = 0.5;
    sendCommand();
}
void GazeboQuadCopter::setPitch(float pitch)
{
    m_Pitch = pitch;
    sendCommand();
}
void GazeboQuadCopter::setRoll(float right)
{
    m_Roll = right;
    sendCommand();
}
void GazeboQuadCopter::setVerticalSpeed(float up)
{
    m_Thrust = up*0.5 + 0.5;
    sendCommand();
}
void GazeboQuadCopter::setYawSpeed(float right)
{
    m_Yaw = right;
    sendCommand();
}
void GazeboQuadCopter::sendCommand()
{
    // std::cout << m_fl << "\t\t"<< m_fr << std::endl << "\tX\n" <<  m_bl<< "\t\t" << m_br << std::endl;
    gazebo::msgs::Set(&msg, ignition::math::Quaterniond(m_Thrust, m_Roll, m_Pitch, m_Yaw));
    // Send the message
    pub->Publish(msg);
}

} // Robots
} // BoBRobotics
