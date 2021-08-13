/*
    Pure Pursuit Controller. The controller takes a list of waypoints as a path to follow,
    then it calculates the optimal turning angle to stay on the path
*/
#pragma once

// BoB robotics includes
#include "common/pose.h"

// Third-party includes
#include "third_party/optional.hpp"
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
                          millimeter_t stoppingDist);

    //! set waypoints which forms a path to be followed
    void setWayPoints(const std::vector<Vector2<millimeter_t>> &wp);

    //! adds to the list of waypoints
    void addWayPoint(const Vector2<millimeter_t> &wayPoint);

    //! sets the lookahead distance. Large values causes the car to cut corners,
    //! where small values makes the car follow the path more closely
    void setLookAheadDistance(const millimeter_t distance);

    //! sets the distance between the car's wheel bases
    void setWheelBaseLength(const millimeter_t length);

    //! sets the stopping distance. If the car is within this distance, the controller stops
    void setStoppingDistance(const millimeter_t distance);

    //! calculates the turning angle needed to follow the path or nullopt if not available
    std::experimental::optional<degree_t>
    getTurningAngle(const Pose2<millimeter_t, radian_t> &robotPose,
                    const std::experimental::optional<Vector2<millimeter_t>> &lookPoint) const;

    //! calculates the look-ahead point the robot follows
    std::experimental::optional<Vector2<millimeter_t>>
    getLookAheadPoint(const Vector2<millimeter_t> &robotPosition,
                      millimeter_t lookAheadDistance) const;

private:
    millimeter_t m_lookAheadDistance;                 // the distance to look ahead
    millimeter_t m_wheelBase;                         // length between wheel bases
    millimeter_t m_stoppingDistance;                  // stopping distance.
    std::vector<Vector2<millimeter_t>> m_wayPoints;   // list of waypoint coordinates

    // computes the turning angle using the lookahead point
    degree_t computeTurningAngle(const Pose2<millimeter_t, radian_t> &robotPose,
                                 const Vector2<millimeter_t> &lookPoint,
                                 millimeter_t lookAheadDistance) const;
}; // PurePursuitController
}  // Robot
}  // BoBRobotics
