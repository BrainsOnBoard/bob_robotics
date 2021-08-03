#pragma once

// BoB robotics includes
#include "hid/joystick.h"

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace Robots {
namespace Ackermann {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::AckermannBase
//----------------------------------------------------------------------------
//! A base class for robots with Ackermann-type steering
template<class Derived>
class AckermannBase
{
protected:
    AckermannBase<Derived>() = default;

public:
    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f)
    {
        joystick.addHandler([this, deadZone](auto &, HID::JAxis axis, float value) {
            auto *derived = static_cast<Derived *>(this);

            if (fabs(value) <= deadZone) {
                value = 0.f;
            }

            if (axis == HID::JAxis::LeftStickVertical) {
                derived->moveForward(-value);
                return true;
            } else if (axis == HID::JAxis::RightStickHorizontal) {
                derived->steer(-value);
                return true;
            } else {
                return false;
            }
        });
    }
}; // AckermannBase
} // Ackermann
} // Robots
} // BoBRobotics
