#pragma once

// BoB robotics includes
#include "hid/joystick.h"
#include "robots/tank/tank_base.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Omni2DBase
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
    // TankBase virtuals
    //------------------------------------------------------------------------
    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f);
    void drive(const HID::Joystick &joystick, float deadZone = 0.25f);
    void tank(float left, float right);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    float getForwards() const;
    float getSideways() const;
    float getTurn() const;

protected:
    void setWheelSpeed(float forward, float sideways, float turn);

private:
    float m_Forward = 0;
    float m_Sideways = 0;
    float m_Turn = 0;

    void drive(float forward, float sideways, float turn, float deadZone);
    bool onJoystickEvent(HID::JAxis axis, float value, float deadZone);
}; // Omni2D
} // Robots
} // BoBRobotics
