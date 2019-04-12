// BoB robotics includes
#include "robots/uav.h"

namespace BoBRobotics {
namespace Robots {

void UAV::moveForward(float speed)
{
    setPitch(speed);
}

void UAV::turnOnTheSpot(float clockwiseSpeed)
{
    setPitch(0.f);
    setRoll(0.f);
    setVerticalSpeed(0.f);
    setYawSpeed(clockwiseSpeed);
}

void UAV::stopMoving()
{
    setPitch(0.f);
    setRoll(0.f);
    setVerticalSpeed(0.f);
    setYawSpeed(0.f);
}

void UAV::addJoystick(HID::Joystick &joystick)
{
    joystick.addHandler([this](HID::JAxis axis, float value) {
        return onAxisEvent(axis, value);
    });
    joystick.addHandler([this](HID::JButton button, bool pressed) {
        return onButtonEvent(button, pressed);
    });
}

bool UAV::onAxisEvent(HID::JAxis axis, float value)
{
    /*
     * setRoll/Pitch etc. all take values between -1 and 1. We cap these
     * values for the joystick code to make the drone more controllable.
     */
    switch (axis) {
    case HID::JAxis::RightStickHorizontal:
        setRoll(value);
        return true;
    case HID::JAxis::RightStickVertical:
        setPitch(-value);
        return true;
    case HID::JAxis::LeftStickVertical:
        setVerticalSpeed(-value);
        return true;
    case HID::JAxis::LeftTrigger:
        setYawSpeed(-value);
        return true;
    case HID::JAxis::RightTrigger:
        setYawSpeed(value);
        return true;
    default:
        // otherwise signal that we haven't handled event
        return false;
    }
}

bool UAV::onButtonEvent(HID::JButton button, bool pressed)
{
    // we only care about button presses
    if (!pressed) {
        return false;
    }

    // A = take off; B = land
    switch (button) {
    case HID::JButton::A:
        takeOff();
        return true;
    case HID::JButton::B:
        land();
        return true;
    default:
        // otherwise signal that we haven't handled event
        return false;
    }
}

} // Robots
} // BoBRobotics
