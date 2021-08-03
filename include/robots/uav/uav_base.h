#pragma once

// BoB robotics includes
#include "hid/joystick.h"

namespace BoBRobotics {
namespace Robots {
template<class Derived>
class UAVBase
{
public:
    void moveForward(float speed)
    {
        setPitch(speed);
    }

    void turnOnTheSpot(float clockwiseSpeed)
    {
        setPitch(0.f);
        setRoll(0.f);
        setVerticalSpeed(0.f);
        setYawSpeed(clockwiseSpeed);
    }

    void stopMoving()
    {
        setPitch(0.f);
        setRoll(0.f);
        setVerticalSpeed(0.f);
        setYawSpeed(0.f);
    }

    void addJoystick(HID::Joystick &joystick)
    {
        joystick.addHandler([this](auto &, HID::JAxis axis, float value) {
            return onAxisEvent(axis, value);
        });
        joystick.addHandler([this](auto &, HID::JButton button, bool pressed) {
            return onButtonEvent(button, pressed);
        });
    }

    void drive(const HID::Joystick &joystick)
    {
        setRoll(joystick.getState(HID::JAxis::RightStickHorizontal));
        setPitch(-joystick.getState(HID::JAxis::RightStickVertical));
        setVerticalSpeed(-joystick.getState(HID::JAxis::LeftStickVertical));
        setYawSpeed(joystick.getState(HID::JAxis::RightTrigger)
                             - joystick.getState(HID::JAxis::LeftTrigger));
    }

private:
    bool onAxisEvent(HID::JAxis axis, float value)
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

    bool onButtonEvent(HID::JButton button, bool pressed)
    {
        // we only care about button presses
        if (!pressed) {
            return false;
        }

        // A = take off; B = land
        auto *derived = static_cast<Derived *>(this);
        switch (button) {
        case HID::JButton::A:
            derived->takeOff();
            return true;
        case HID::JButton::B:
            derived->land();
            return true;
        default:
            // otherwise signal that we haven't handled event
            return false;
        }
    }

#define DERIVED_FUNC(NAME)                         \
    void NAME(float value)                         \
    {                                              \
        static_cast<Derived *>(this)->NAME(value); \
    }

    DERIVED_FUNC(setPitch)
    DERIVED_FUNC(setRoll)
    DERIVED_FUNC(setYawSpeed)
    DERIVED_FUNC(setVerticalSpeed)

#undef DERIVED_FUNC
};
} // Robots
} // BoBRobotics
