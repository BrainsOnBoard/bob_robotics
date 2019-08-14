// BoB robotics includes
#include "robots/control/pure_pursuit_controller.h"

using namespace units::angle;
using namespace units::length;

// sign function
template <typename T>static int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace BoBRobotics {
namespace Robots {

PurePursuitController::PurePursuitController() {}

PurePursuitController::PurePursuitController(millimeter_t lookahead,
                                             millimeter_t wheelBaseLength,
                                             millimeter_t stopping_dist)
  : m_lookAheadDistance(lookahead)
  , m_wheelBase(wheelBaseLength)
  , m_stoppingDistance(stopping_dist)
{
}

//! set waypoints which forms a path to be followed
void PurePursuitController::setWayPoints(const std::vector<Vector2<millimeter_t>> &wp) {
    m_wayPoints = wp;
}

//! adds to the list of waypoints
void PurePursuitController::addToWayPoint(const Vector2<millimeter_t> wayPoint) {
    m_wayPoints.push_back(wayPoint);
}

//! sets the lookahead distance. Large values causes the car to cut corners,
//! where small values makes the car follow the path more closely
void PurePursuitController::setlookAheadDistance(const millimeter_t distance) {
    m_lookAheadDistance = distance;
}

//! sets the distance between the car's wheel bases
void PurePursuitController::setWheelBaseLength(const millimeter_t length) {
    m_wheelBase = length;
}

//! sets the stopping distance. If the car is within this distance, the controller stops
void PurePursuitController::setStoppingDistance(const millimeter_t distance) {
    m_stoppingDistance = distance;
}

//! calculates the turning angle needed to follow the path, returns true if there is a valid angle.
bool PurePursuitController::getTurningAngle(const millimeter_t x, const millimeter_t y, const radian_t heading, degree_t &turningAngle) {

    using namespace units::math;
    Vector2<millimeter_t> lookPoint;                                           // lookahead point
    bool didGetPoint = getLookAheadPoint(x,y, m_lookAheadDistance, lookPoint); // did we get a valid lookahead point (and also calculate the point)
    const millimeter_t wx = m_wayPoints.back().x();                            // last wp x
    const millimeter_t wy = m_wayPoints.back().y();                            // last wp y
    const millimeter_t dist = sqrt( pow<2>(wx-x) + pow<2>(wy-y) );             // distance to last point

    // lookAhead distance changes based on the distance between robot and last waypint
    millimeter_t  lookAheadDistance = m_lookAheadDistance;
    // if we arrived to the last waypoint, stop
    if (dist < m_stoppingDistance) return false;

    // if we have a valid lookahead point
    if (didGetPoint) {
        // if the distance to the last point is less than the lookahead distance, set
        // the lookaheadDistance to the distance between robot and last point
        if (dist <= m_lookAheadDistance && !m_wayPoints.empty()) {
            lookAheadDistance = dist;
            lookPoint.x() = wx;
            lookPoint.y() = wy;
        }
        turningAngle = computeTurningAngle(x,y, lookPoint.x(), lookPoint.y(), lookAheadDistance, heading);
        return true;
    }
    // if we don't have a valid lookahead point and we are closer the last waypoint than the lookahead distance
    else if (!didGetPoint && dist < m_lookAheadDistance) {
        lookAheadDistance = dist;
        turningAngle = computeTurningAngle(x,y, wx, wy, lookAheadDistance, heading);
        return true;
    }

    return false;
}

//! calculates the look-ahead point the robot follows. returns true if there is a valid point
bool PurePursuitController::getLookAheadPoint(const millimeter_t x, const millimeter_t y, const millimeter_t r, Vector2<millimeter_t> &lookaheadPoint) {
    using namespace units::math;

    if (m_wayPoints.size() > 1) {
        bool didGetIntersection = false;
        for (unsigned int i = 0; i < m_wayPoints.size()-1; i++) {

            // path points
            const Vector2<millimeter_t> segmentStart = m_wayPoints.at(i);
            const Vector2<millimeter_t> segmentEnd = m_wayPoints.at(i+1);

            const auto p1x = segmentStart.x() - x;
            const auto p1y = segmentStart.y() - y;
            const auto p2x = segmentEnd.x() - x;
            const auto p2y = segmentEnd.y() - y;

            // intersection point with circle
            const auto dx = p2x - p1x;
            const auto dy = p2y - p1y;
            const auto d = sqrt(dx * dx + dy * dy);
            const auto D = p1x * p2y - p2x * p1y;

            // if the discriminant is zero -> no intersection
            const auto discriminant = r * r * d * d - D * D;
            if (discriminant.value() < 0) continue;

            // x components of the intersection point
            const auto x1 = (D * dy + sgn(dy.value()) * dx * sqrt(discriminant)) / (d*d);
            const auto x2 = (D * dy - sgn(dy.value()) * dx * sqrt(discriminant)) / (d*d);
            // y components of the intersection point
            const auto y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / (d*d);
            const auto y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / (d*d);

            // if there is 2 intersections possible with the circle, we select the second one,
            // as that will be closer to the end point of the segment
            const bool validIntersection1 = (min(p1x, p2x) < x1 && x1 < max(p1x, p2x)) ||
                                        (min(p1y, p2y) < y1 && y1 < max(p1y,p2y));
            const bool validIntersection2 = (min(p1x, p2x) < x2 && x2 < max(p1x, p2x)) ||
                                        (min(p1y, p2y) < y2 && y2 < max(p1y,p2y));

            // we always want the latest path segment point so if we have a
            // valid point, we delete the previous point
            if (validIntersection1) {
                lookaheadPoint.x() = x1 + x;
                lookaheadPoint.y() = y1 + y;
                didGetIntersection = true;
            }

            // if there is a valid 2. intersection point, we keep that and remove
            // the first one
            if (validIntersection2) {
                if (lookaheadPoint.isnan() || fabs(x1 - p2x) > fabs(x2 - p2x) ||
                    fabs(y1 - p2y) > fabs(y2 - p2y)) {
                    lookaheadPoint.x() = x2 + x;
                    lookaheadPoint.y() = y2 + y;
                    didGetIntersection = true;
                }
            }
        }
        if (didGetIntersection) return true;
    }
    return false;
}

// computes the turning angle using the lookahead point
degree_t PurePursuitController::computeTurningAngle(const millimeter_t xrobot, const millimeter_t yrobot
                            ,const millimeter_t xlookahead, const millimeter_t ylookahead
                            ,const millimeter_t lookAheadDistance ,const radian_t heading) {

    using namespace units::math;
    // calculating bearing [robot-lookahead point]
    const auto dx = xlookahead-xrobot;
    const auto dy = ylookahead-yrobot;
    const degree_t bearing = atan2(dy,dx)-heading;

    // calculating turning angle
    const auto xlength = sin(bearing) * lookAheadDistance;

    // calculating arc to turn
    const auto kb = (2*xlength*m_wheelBase)/(pow<2>(lookAheadDistance));
    const degree_t turningAngle = atan(kb);
    return turningAngle;
}

}  // Robot
}  // BoBRobotics




