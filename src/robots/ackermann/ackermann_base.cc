// BoB robotics includes
#include "common/macros.h"
#include "robots/ackermann/ackermann_base.h"

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace Robots {
AckermannBase::~AckermannBase()
{}

void
AckermannBase::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler([this, deadZone](HID::JAxis axis, float value) {
        if (fabs(value) <= deadZone) {
            value = 0.f;
        }
        if (axis == HID::JAxis::LeftStickVertical) {
            moveForward(-value);
            return true;
        } else if (axis == HID::JAxis::RightStickHorizontal) {
            steer(-value);
            return true;
        } else {
            return false;
        }
    });
}

BOB_NOT_IMPLEMENTED(
    void
    AckermannBase::move(units::velocity::meters_per_second_t,
                    units::angle::degree_t)
)
} // Robots
} // BoBRobotics
