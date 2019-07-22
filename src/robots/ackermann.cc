// BoB robotics includes
#include "common/macros.h"
#include "robots/ackermann.h"

namespace BoBRobotics {
namespace Robots {
Ackermann::~Ackermann()
{}

#ifdef USE_BOB_HID
void
Ackermann::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler([this](HID::JAxis axis, float value) {
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
#endif

BOB_NOT_IMPLEMENTED(
    void
    Ackermann::move(units::velocity::meters_per_second_t,
                    units::angle::degree_t)
)
} // Robots
} // BoBRobotics
