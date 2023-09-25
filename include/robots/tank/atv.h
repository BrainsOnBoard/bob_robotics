#pragma once
#ifndef _WIN32

// BoB robotics includes
#include "common/robo_claw.h"
#include "robots/tank/tank_base.h"

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Robots {
namespace Tank {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Tank::ATV
//----------------------------------------------------------------------------
//! An interface for large wheeled, all-terrain robot with two RoboClaw motor controllers
class ATV : public TankBase<ATV>
{
    friend TankBase<ATV>;
public:
    ATV(const char *frontPath = "//dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A_10-if00",
        const char *rearPath = "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A_20-if00",
        uint8_t frontAddress = 0x80, uint8_t rearAddress = 0x80);

    //----------------------------------------------------------------------------
    // TankBase virtuals
    //----------------------------------------------------------------------------
    ~ATV();

    static constexpr auto getMaximumSpeed()
    {
        return units::velocity::meters_per_second_t{ 0.11 };
    }

    static constexpr auto getRobotWidth()
    {
        return units::length::millimeter_t{ 104 };
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    RoboClaw &getFrontController(){ return m_FrontController; }
    RoboClaw &getRearController(){ return m_RearController; }

private:
    void tankInternal(float left, float right);

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    RoboClaw m_FrontController;
    RoboClaw m_RearController;
}; // Norbot
} // Tank
} // Robots
} // BoBRobotics
#endif // __linux__
