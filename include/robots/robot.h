#pragma once

// BoB robotics includes
#include "hid/joystick.h"

// Forward declarations
namespace BoBRobotics
{
namespace Net
{
class Connection;
}
}

namespace BoBRobotics {
namespace Robots {
//! A generic abstract class for robots and other moveable agents
class Robot
{
public:
    virtual ~Robot()
    {}

    /**!
     * \brief Move forward at the specified relative speed
     *
     * Values must be between -1 and 1 inclusive.
     */
    virtual void moveForward(float speed) = 0;

    /**!
     * \brief Stop moving forward and start turning at the specified relative speed
     *
     * Values must be between -1 and 1 inclusive.
     */
    virtual void turnOnTheSpot(float clockwiseSpeed) = 0;

    //! Stop the robot moving
    virtual void stopMoving() = 0;
    
    //! Add a handler to the joystick to drive robot
    virtual void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f) = 0;
    
    //! Drive the robot using the current joystick state
    virtual void drive(const HID::Joystick &joystick, float deadZone = 0.25f) = 0;
    
    //! Add a handler to the connection to drive robot
    virtual void readFromNetwork(Net::Connection &connection) = 0;
    
    //! Remove and connection handlers that drive this robot
    virtual void stopReadingFromNetwork() = 0;
}; // Robot
} // Robots
} // BoBRobotics
