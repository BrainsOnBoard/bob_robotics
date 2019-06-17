// BoB robotics includes
#include "robots/ackermann.h"

namespace BoBRobotics {
namespace Robots {
#ifdef USE_BOB_HID
void
Ackermann::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler([this](HID::JAxis axis, float value) {
        if (axis == HID::JAxis::LeftStickVertical) {
            moveForward(-value);
            return true;
        } else if (axis == HID::JAxis::RightStickHorizontal) {
            steer(value);
            return true;
        } else {
            return false;
        }
    });
}
#endif
}
}
