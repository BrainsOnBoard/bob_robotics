#pragma once

// BoB robotics includes
#include "hid/joystick.h"

namespace BoBRobotics {
namespace Robots {
namespace Omni2D {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Omni2D::Omni2DBase
//----------------------------------------------------------------------------
// Interface for driving Omni2D-like wheeled robots
template<class Derived>
class Omni2DBase
{
public:
    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f)
    {
        joystick.addHandler(
                [this, deadZone](auto &joystick, HID::JAxis axis, float) {
                    return onJoystickEvent(joystick, axis, deadZone);
                });
    }

    void drive(const HID::JoystickBase<HID::JAxis, HID::JButton> &joystick,
               float deadZone = 0.25f)
    {
        drive(-joystick.getState(HID::JAxis::LeftStickVertical),
              joystick.getState(HID::JAxis::LeftStickHorizontal),
              joystick.getState(HID::JAxis::RightStickHorizontal),
              deadZone);
    }

    void moveForward(float speed)
    {
        omni2D(speed, 0.f, 0.f);
    }

    void turnOnTheSpot(float clockwiseSpeed)
    {
        // **TODO**: Check that this really is clockwise
        omni2D(0.f, 0.f, clockwiseSpeed);
    }

    void stopMoving()
    {
        omni2D(0.f, 0.f, 0.f);
    }

private:
    void drive(float forward, float sideways, float turn, float deadZone)
    {
        const bool deadForward = (fabs(forward) < deadZone);
        const bool deadSideways = (fabs(sideways) < deadZone);
        const bool deadTurn = (fabs(turn) < deadZone);

        // Drive motor
        omni2D(forward * !deadForward, sideways * !deadSideways, turn * !deadTurn);
    }

    void omni2D(float forward, float sideways, float turn)
    {
        auto *derived = static_cast<Derived *>(this);
        derived->omni2D(forward, sideways, turn);
    }

    bool onJoystickEvent(HID::JoystickBase<HID::JAxis, HID::JButton> &joystick,
                         HID::JAxis axis, float deadZone)
    {
        if (axis == HID::JAxis::LeftStickVertical
            || axis == HID::JAxis::LeftStickHorizontal
            || axis == HID::JAxis::RightStickHorizontal) {

            // drive robot with joystick
            drive(joystick, deadZone);
            return true;
        }

        return false;
    }
}; // Omni2D
} // Omni2D
} // Robots
} // BoBRobotics
