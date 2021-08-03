#pragma once

// BoB robotics includes
#include "hid/joystick.h"
#include "robots/tank/tank_base.h"

namespace BoBRobotics {
namespace Robots {
namespace Omni2D {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Omni2D::Omni2DBase
//----------------------------------------------------------------------------
// Interface for driving Omni2D-like wheeled robots
class Omni2DBase
  : public Tank::TankBase<Omni2DBase>
{
public:
    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual void omni2D(float forward, float sideways, float turn);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f);
    void drive(const HID::JoystickBase<HID::JAxis, HID::JButton> &joystick,
               float deadZone = 0.25f);
    void tank(float left, float right);

private:
    void drive(float forward, float sideways, float turn, float deadZone);
    bool onJoystickEvent(HID::JoystickBase<HID::JAxis, HID::JButton> &,
                         HID::JAxis axis, float value, float deadZone);
}; // Omni2D
} // Omni2D
} // Robots
} // BoBRobotics
