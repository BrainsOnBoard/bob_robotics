#ifndef _WIN32
// BoB robotics includes
#include "common/robo_claw.h"
#include "common/macros.h"


//----------------------------------------------------------------------------
// BoBRobotics::RoboClaw
//----------------------------------------------------------------------------
namespace BoBRobotics {
Roboclaw::Roboclaw(const char *path, uint8_t address, int maxRetry = 2)
:   m_SerialInterface(path), m_Address(address), m_MaxRetry(maxRetry)
{
}
//----------------------------------------------------------------------------
void Roboclaw::setMotor1Speed(float throttle)
{
    const bool forward = (throttle > 0.0f);
    const uint8_t byte = (uint8_t)std::round(255.0 * std::min(1.0f, forward ? throttle : -throttle));

    writeN(forward ? Command::M1FORWARD : Command::M1BACKWARD, byte)
}
//----------------------------------------------------------------------------
void Roboclaw::setMotor2Speed(float throttle)
{
    const bool forward = (throttle > 0.0f);
    const uint8_t byte = (uint8_t)std::round(255.0 * std::min(1.0f, forward ? throttle : -throttle));

    writeN(forward ? Command::M2FORWARD : Command::M2BACKWARD, byte)
}
//----------------------------------------------------------------------------
}   // namespace BoBRobotics
#endif   // _WIN32
