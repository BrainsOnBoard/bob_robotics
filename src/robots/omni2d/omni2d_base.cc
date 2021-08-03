// BoB robotics includes
#include "robots/omni2d/omni2d_base.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Robots {
namespace Omni2D {

void
Omni2DBase::omni2D(float forward, float sideways, float turn)
{
    LOGI << "Dummy motor: forward: " << forward << "; sideways: " << sideways << "; turn: " << turn;
}

void
Omni2DBase::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler(
            [this, deadZone](auto &joystick, HID::JAxis axis, float value) {
                return onJoystickEvent(joystick, axis, value, deadZone);
            });
}

void
Omni2DBase::drive(const HID::JoystickBase<HID::JAxis, HID::JButton> &joystick,
                  float deadZone)
{
    drive(-joystick.getState(HID::JAxis::LeftStickVertical),
          joystick.getState(HID::JAxis::LeftStickHorizontal),
          joystick.getState(HID::JAxis::RightStickHorizontal),
          deadZone);
}

void Omni2DBase::tank(float left, float right)
{
    // Implement tank controls in terms of omni
    omni2D((left + right) / 2.0f, 0.0f, (left - right) / 2.0f);
}

void
Omni2DBase::drive(float forward, float sideways, float turn, float deadZone)
{
    const bool deadForward = (fabs(forward) < deadZone);
    const bool deadSideways = (fabs(sideways) < deadZone);
    const bool deadTurn = (fabs(turn) < deadZone);

    // Drive motor
    omni2D(forward * !deadForward, sideways * !deadSideways, turn * !deadTurn);
}

bool
Omni2DBase::onJoystickEvent(HID::JoystickBase<HID::JAxis, HID::JButton> &joystick,
                            HID::JAxis axis, float, float deadZone)
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

} // Omni2D
} // Robots
} // BoBRobotics
