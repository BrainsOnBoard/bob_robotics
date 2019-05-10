/*
    Pure Pursuit Controller. The controller takes a list of waypoints as a path to follow,
    then it calculates the optimal turning angle to stay on the path
*/
#pragma once

// BoB robotics includes
#include "common/pose.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
namespace Robots {
/**!
 * \brief The controller takes a list of waypoints as a path to follow,
 *        then it calculates the optimal turning angle to stay on the path.
 */
class PurePursuitController {

    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using radian_t = units::angle::radian_t;

public:
    PurePursuitController();

    PurePursuitController(millimeter_t lookahead,
                          millimeter_t wheelBaseLength,
                          millimeter_t stopping_dist);

    //! set waypoints which forms a path to be followed
    void setWayPoints(const std::vector<Vector2<millimeter_t>> &wp);

    //! adds to the list of waypoints
    void addToWayPoint(const Vector2<millimeter_t> wayPoint);

    //! sets the lookahead distance. Large values causes the car to cut corners,
    //! where small values makes the car follow the path more closely
    void setlookAheadDistance(const millimeter_t distance);

    //! sets the distance between the car's wheel bases
    void setWheelBaseLength(const millimeter_t length);

    //! sets the stopping distance. If the car is within this distance, the controller stops
    void setStoppingDistance(const millimeter_t distance);

    //! calculates the turning angle needed to follow the path, returns true if there is a valid angle.
    bool getTurningAngle(const millimeter_t x, const millimeter_t y, const radian_t heading, degree_t &turningAngle);

    //! calculates the look-ahead point the robot follows. returns true if there is a valid point
    bool getLookAheadPoint(const millimeter_t x, const millimeter_t y, const millimeter_t r, Vector2<millimeter_t> &lookaheadPoint);

private:

    millimeter_t m_lookAheadDistance;                 // the distance to look ahead
    millimeter_t m_wheelBase;                         // length between wheel bases
    millimeter_t m_stoppingDistance;                  // stopping distance.
    std::vector<Vector2<millimeter_t>> m_wayPoints;   // list of waypoint coordinates

    // computes the turning angle using the lookahead point
    degree_t computeTurningAngle(const millimeter_t xrobot,
                                 const millimeter_t yrobot,
                                 const millimeter_t xlookahead,
                                 const millimeter_t ylookahead,
                                 const millimeter_t lookAheadDistance,
                                 const radian_t heading);
}; // PurePursuitController
}  // Robot
}  // BoBRobotics
