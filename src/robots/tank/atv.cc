#ifndef _WIN32
// BoB robotics includes
#include "common/robo_claw.h"
#include "robots/tank/atv.h"

namespace BoBRobotics {
namespace Robots {
namespace Tank {

ATV::ATV(const char *frontPath, const char *rearPath, uint8_t frontAddress, uint8_t rearAddress)
:   m_FrontController(frontPath, frontAddress), m_RearController(rearPath, rearAddress)
{
    // Sometimes robots get stuck driving, so let's stop it if we need to
    stopMoving();
}

//----------------------------------------------------------------------------
// TankBase virtuals
//----------------------------------------------------------------------------
ATV::~ATV()
{
    stopMoving();
}

void ATV::tankInternal(float left, float right)
{
    m_FrontController.setMotor1Speed(-left);
    m_FrontController.setMotor2Speed(right);

    m_RearController.setMotor1Speed(-left);
    m_RearController.setMotor2Speed(-right);
}

} // Tank
} // Robots
} // BoBRobotics
#endif
