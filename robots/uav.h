#pragma once

// BoB robotics includes
#include "../hid/joystick.h"
#include "robot.h"

namespace BoBRobotics {
namespace Robots {
class UAV
  : public Robot
{
public:
    virtual ~UAV()
    {}

    virtual void takeOff() = 0;
    virtual void land() = 0;
    virtual void setPitch(float pitch) = 0;
    virtual void setRoll(float right) = 0;
    virtual void setVerticalSpeed(float up) = 0;
    virtual void setYawSpeed(float right) = 0;

    //! An alias for setPitch
    virtual void moveForward(float speed) override
    {
        setPitch(speed);
    }

    //! Sets the yaw speed, stopping motion in all other axes
    virtual void turn(float clockwiseSpeed) override
    {
        setPitch(0.f);
        setRoll(0.f);
        setVerticalSpeed(0.f);
        setYawSpeed(clockwiseSpeed);
    }

    virtual void stopMoving() override
    {
        setPitch(0.f);
        setRoll(0.f);
        setVerticalSpeed(0.f);
        setYawSpeed(0.f);
    }

    //! Start controlling this drone with a joystick
    void addJoystick(HID::Joystick &joystick)
    {
        joystick.addHandler([this](HID::JAxis axis, float value) {
            return onAxisEvent(axis, value);
        });
        joystick.addHandler([this](HID::JButton button, bool pressed) {
            return onButtonEvent(button, pressed);
        });
    }

private:
    // Handle joystick axis events
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

    // Handle joystick button events
    bool onButtonEvent(HID::JButton button, bool pressed)
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
};
} // Robots
} // BoBRobotics