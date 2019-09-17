#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "video/input.h"

// Standard C++ includes
#include <memory>

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

    //! Get the default camera associated with this robot
    BOB_NOT_IMPLEMENTED(virtual std::unique_ptr<Video::Input> getCamera())
}; // Robot
} // Robots
} // BoBRobotics
