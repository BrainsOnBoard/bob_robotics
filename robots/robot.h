#pragma once

// BoB robotics includes
#include "../hid/joystick.h"

// Third-party includes
#include "../third_party/units.h"

// Standard C++ includes
#include <stdexcept>

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

    //! Start controlling this Robot with a joystick
    virtual void addJoystick(HID::Joystick &)
    {
        throw std::runtime_error("addJoystick() is not implemented for this Robot");
    }

    virtual units::angular_velocity::degrees_per_second_t getMaximumTurnSpeed()
    {
        throw std::runtime_error("getMaximumTurnSpeed() is not implemented for this Robot");
    }
}; // Robot
} // Robots
} // BoBRobotics
