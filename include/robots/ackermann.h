#pragma once

// BoB robotics includes
#ifdef USE_BOB_HID
#include "hid/joystick.h"
#endif

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Ackermann
//----------------------------------------------------------------------------
//! A base class for robots with Ackermann-type steering
class Ackermann
{
public:
#ifdef USE_BOB_HID
    virtual void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f);
#endif

    virtual units::angle::degree_t getMaximumTurn() const = 0;
    virtual void move(units::velocity::meters_per_second_t velocity,
                      units::angle::degree_t steeringAngle) = 0;
    virtual void moveForward(float speed) = 0;
    virtual void steer(float left) = 0;
    virtual void steer(units::angle::degree_t left) = 0;
    virtual void stopMoving() = 0;
}; // Ackermann
} // Robots
} // BoBRobotics
