// BoB robotics includes
#include "common/macros.h"
#include "robots/control/pure_pursuit_controller.h"

// Standard C++ includes
#include <cmath>

using namespace units::angle;
using namespace units::length;

// sign function
template<typename T>
static int
sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

namespace BoBRobotics {
namespace Robots {

PurePursuitController::PurePursuitController()
{}

PurePursuitController::PurePursuitController(millimeter_t lookAhead,
                                             millimeter_t wheelBaseLength,
                                             millimeter_t stoppingDist)
  : m_lookAheadDistance(lookAhead)
  , m_wheelBase(wheelBaseLength)
  , m_stoppingDistance(stoppingDist)
{
}

//! adds to the list of waypoints
void
PurePursuitController::addWayPoint(const Vector2<millimeter_t> &wayPoint)
{
    /*
     * Check that the way point is a real number. This is not a given, as e.g.
     * we use NaNs to indicate that GPS data was not available in
     * dataset_recorder.
     */
    BOB_ASSERT(std::isfinite(wayPoint.x().value()) && std::isfinite(wayPoint.y().value()));

    m_wayPoints.push_back(wayPoint);
}

//! sets the lookahead distance. Large values causes the car to cut corners,
//! where small values makes the car follow the path more closely
void
PurePursuitController::setLookAheadDistance(millimeter_t distance)
{
    m_lookAheadDistance = distance;
}

//! sets the distance between the car's wheel bases
void
PurePursuitController::setWheelBaseLength(millimeter_t length)
{
    m_wheelBase = length;
}

//! sets the stopping distance. If the car is within this distance, the controller stops
void
PurePursuitController::setStoppingDistance(millimeter_t distance)
{
    m_stoppingDistance = distance;
}

//! calculates the turning angle needed to follow the path, returns true if there is a valid angle.
std::experimental::optional<degree_t>
PurePursuitController::getTurningAngle(const Pose2<millimeter_t, radian_t> &robotPose,
                                       const std::experimental::optional<Vector2<millimeter_t>> &lookPoint) const
{

    using namespace units::math;

    BOB_ASSERT(!m_wayPoints.empty());
    const auto &lastWaypoint = m_wayPoints.back();
    const millimeter_t dist = lastWaypoint.distance2D(robotPose); // distance to last point

    // if we arrived to the last waypoint, stop
    // **TODO**: Should we be doing this check here?
    if (dist < m_stoppingDistance) {
        return std::experimental::nullopt;
    }

    // if the distance to the last point is less than the lookahead distance, set
    // the lookaheadDistance to the distance between robot and last point
    if (dist <= m_lookAheadDistance) {
        return computeTurningAngle(robotPose, lastWaypoint, dist);
    }

    // if we have a valid lookahead point
    if (lookPoint) {
        return computeTurningAngle(robotPose, lookPoint.value(), m_lookAheadDistance);
    }

    // if we don't have a valid lookahead point and we are closer the last waypoint than the lookahead distance
    return std::experimental::nullopt;
}

std::experimental::optional<Vector2<millimeter_t>>
PurePursuitController::getLookAheadPoint(const Vector2<millimeter_t> &robotPosition) const
{
    using namespace units::math;

    BOB_ASSERT(m_wayPoints.size() > 1);

    std::experimental::optional<Vector2<millimeter_t>> lookAheadPoint;
    for (unsigned int i = 0; i < m_wayPoints.size() - 1; i++) {
        // path points
        const Vector2<millimeter_t> segmentStart = m_wayPoints.at(i);
        const Vector2<millimeter_t> segmentEnd = m_wayPoints.at(i + 1);

        const auto p1x = segmentStart.x() - robotPosition.x();
        const auto p1y = segmentStart.y() - robotPosition.y();
        const auto p2x = segmentEnd.x() - robotPosition.x();
        const auto p2y = segmentEnd.y() - robotPosition.y();

        // intersection point with circle
        const auto dx = p2x - p1x;
        const auto dy = p2y - p1y;
        const auto d = sqrt(dx * dx + dy * dy);
        const auto D = p1x * p2y - p2x * p1y;

        // if the discriminant is zero -> no intersection
        const auto discriminant = m_lookAheadDistance * m_lookAheadDistance * d * d - D * D;
        if (discriminant.value() < 0) {
            continue;
        }

        // x components of the intersection point
        const auto x1 = (D * dy + sgn(dy.value()) * dx * sqrt(discriminant)) / (d * d);
        const auto x2 = (D * dy - sgn(dy.value()) * dx * sqrt(discriminant)) / (d * d);
        // y components of the intersection point
        const auto y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / (d * d);
        const auto y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / (d * d);

        // if there is 2 intersections possible with the circle, we select the second one,
        // as that will be closer to the end point of the segment
        const bool validIntersection1 = (min(p1x, p2x) < x1 && x1 < max(p1x, p2x)) ||
                                        (min(p1y, p2y) < y1 && y1 < max(p1y, p2y));
        const bool validIntersection2 = (min(p1x, p2x) < x2 && x2 < max(p1x, p2x)) ||
                                        (min(p1y, p2y) < y2 && y2 < max(p1y, p2y));

        // we always want the latest path segment point so if we have a
        // valid point, we delete the previous point
        if (validIntersection1) {
            lookAheadPoint.emplace(x1 + robotPosition.x(), y1 + robotPosition.y());
        }

        // if there is a valid 2. intersection point, we keep that and remove
        // the first one
        if (validIntersection2 && (!lookAheadPoint ||
            fabs(x1 - p2x) > fabs(x2 - p2x) || fabs(y1 - p2y) > fabs(y2 - p2y)))
        {
            lookAheadPoint.emplace(x2 + robotPosition.x(), y2 + robotPosition.y());
        }
    }

    return lookAheadPoint;
}

degree_t
PurePursuitController::computeTurningAngle(const Pose2<millimeter_t, radian_t> &robotPose,
                                           const Vector2<millimeter_t> &lookAheadPoint,
                                           millimeter_t lookAheadDistance) const
{

    using namespace units::math;
    // calculating bearing [robot-lookahead point]
    const auto dx = lookAheadPoint.x() - robotPose.x();
    const auto dy = lookAheadPoint.y() - robotPose.y();
    const auto bearing = atan2(dy, dx) - robotPose.yaw();

    // calculating turning angle
    const auto xlength = sin(bearing) * lookAheadDistance;

    // calculating arc to turn
    const auto kb = (2 * xlength * m_wheelBase) / (pow<2>(lookAheadDistance));
    return atan(kb);
}

} // Robot
} // BoBRobotics
