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
    virtual void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f) override;

    //! Drive the robot using the current joystick state
    virtual void drive(const HID::Joystick &joystick, float deadZone = 0.25f) override;

    //! Add a handler to the connection to drive robot
    virtual void readFromNetwork(Net::Connection &connection) override;

    //! Remove and connection handlers that drive this robot
    virtual void stopReadingFromNetwork() override;

private:
    // Handle joystick axis events
    bool onAxisEvent(HID::JAxis axis, float value);

    // Handle joystick button events
    bool onButtonEvent(HID::JButton button, bool pressed);
};
} // Robots
} // BoBRobotics
