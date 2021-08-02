#pragma once

// BoB robotics includes
#include "hid/joystick.h"

namespace BoBRobotics {
namespace Robots {
class UAV
{
public:
    virtual ~UAV();

    virtual void takeOff() = 0;
    virtual void land() = 0;
    virtual void setPitch(float pitch) = 0;
    virtual void setRoll(float right) = 0;
    virtual void setVerticalSpeed(float up) = 0;
    virtual void setYawSpeed(float right) = 0;

    //! An alias for setPitch
    virtual void moveForward(float speed);

    //! Sets the yaw speed, stopping motion in all other axes
    virtual void turnOnTheSpot(float clockwiseSpeed);

    virtual void stopMoving();

    //! Start controlling this drone with a joystick
    virtual void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f);

    //! Drive the robot using the current joystick state
    virtual void drive(const HID::Joystick &joystick, float deadZone = 0.25f);

private:
    // Handle joystick axis events
    bool onAxisEvent(HID::JAxis axis, float value);

    // Handle joystick button events
    bool onButtonEvent(HID::JButton button, bool pressed);
};
} // Robots
} // BoBRobotics
