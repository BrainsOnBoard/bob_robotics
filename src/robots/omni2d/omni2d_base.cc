// BoB robotics includes
#include "robots/omni2d/omni2d_base.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Robots {

void
Omni2DBase::omni2D(float forward, float sideways, float turn)
{
    LOGI << "Dummy motor: forward: " << forward << "; sideways: " << sideways << "; turn: " << turn;
}

void
Omni2DBase::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler(
            [this, deadZone](auto &, HID::JAxis axis, float value) {
                return onJoystickEvent(axis, value, deadZone);
            });
}

void
Omni2DBase::drive(const HID::Joystick &joystick, float deadZone)
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

float
Omni2DBase::getForwards() const
{
    return m_Forward;
}

float
Omni2DBase::getSideways() const
{
    return m_Sideways;
}

float
Omni2DBase::getTurn() const
{
    return m_Turn;
}

void Omni2DBase::setWheelSpeed(float forward, float sideways, float turn)
{
    m_Forward = forward;
    m_Sideways = sideways;
    m_Turn = turn;
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
Omni2DBase::onJoystickEvent(HID::JAxis axis, float value, float deadZone)
{
    float forward = m_Forward;
    float sideways = m_Sideways;
    float turn = m_Turn;
    switch (axis) {
    case HID::JAxis::LeftStickVertical:
        forward = -value;
        break;
    case HID::JAxis::LeftStickHorizontal:
        sideways = value;
        break;
    case HID::JAxis::RightStickHorizontal:
        turn = value;
        break;
    default:
        return false;
    }

    // drive robot with joystick
    drive(forward, sideways, turn, deadZone);
    return true;
}
} // Robots
} // BoBRobotics
