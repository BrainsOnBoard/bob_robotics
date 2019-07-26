#pragma once

// BoB robotics includes
#include "hid/joystick.h"
#include "robot.h"

namespace BoBRobotics {
namespace Robots {
class UAV
  : public Robot
{
public:
    virtual void takeOff() = 0;
    virtual void land() = 0;
    virtual void setPitch(float pitch) = 0;
    virtual void setRoll(float right) = 0;
    virtual void setVerticalSpeed(float up) = 0;
    virtual void setYawSpeed(float right) = 0;

    //! An alias for setPitch
    virtual void moveForward(float speed) override;

    //! Sets the yaw speed, stopping motion in all other axes
    virtual void turnOnTheSpot(float clockwiseSpeed) override;

    virtual void stopMoving() override;

    //! Start controlling this drone with a joystick
    void addJoystick(HID::Joystick &joystick);

private:
    // Handle joystick axis events
    bool onAxisEvent(HID::JAxis axis, float value);

    // Handle joystick button events
    bool onButtonEvent(HID::JButton button, bool pressed);
};
} // Robots
} // BoBRobotics
