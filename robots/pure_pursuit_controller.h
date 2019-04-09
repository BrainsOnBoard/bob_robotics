/*
    Pure Pursuit Controller. The controller takes a list of waypoints as a path to follow,
    then it calculates the optimal turning angle to stay on the path
*/

// BoB robotics includes
#include "common/main.h"
#include "robots/car_display.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>



namespace BoBRobotics {
namespace Robots {


using namespace units::literals;

class PurePursuitController {

    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using radian_t = units::angle::radian_t;
   
public:

    PurePursuitController() {

    }

    PurePursuitController(millimeter_t lookahead, millimeter_t wheelBaseLength) : 
                          m_lookAheadDistance(lookahead), m_wheelBase(wheelBaseLength) {

    }

    //! set waypoints which forms a path to be followed
    void setWayPoints(const std::vector<Vector2<millimeter_t>> &wp) {
        m_wayPoints = wp;
    }

    //! adds to the list of waypoints
    void addToWayPoint(const Vector2<millimeter_t> wayPoint) {
        m_wayPoints.push_back(wayPoint);
    }

    //! sets the lookahead distance. Large values causes the car to cut corners,
    //! where small values makes the car follow the path more closely
    void setlookAheadDistance(const millimeter_t distance) {
        m_lookAheadDistance = distance;
    }

    //! sets the distance between the car's wheel bases
    void setWheelBaseLength(const millimeter_t length) {
        m_wheelBase = length;
    }
  
    //! calculates the turning angle needed to follow the path 
    degree_t getTurningAngle(const millimeter_t x, const millimeter_t y, const radian_t heading) {
        Vector2<millimeter_t> lookPoint = getLookAheadPoint(x,y, m_lookAheadDistance);
        if (!lookPoint.isnan()) return computeTurningAngle(x,y, lookPoint.x(), lookPoint.y(), heading);
        return 0_deg; 
    }

    //! calculates the look-ahead point the robot follows
    Vector2<millimeter_t> getLookAheadPoint(const millimeter_t x, const millimeter_t y, const millimeter_t r) {
        using namespace units::math;

        Vector2<millimeter_t> lookaheadVector;
        if (m_wayPoints.size() > 1) {
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
                    lookaheadVector.x() = x1 + x;
                    lookaheadVector.y() = y1 + y;
                }

                // if there is a valid 2. intersection point, we keep that and remove
                // the first one
                if (validIntersection2) {
                    if (lookaheadVector.isnan() || fabs(x1 - p2x) > fabs(x2 - p2x) ||
                        fabs(y1 - p2y) > fabs(y2 - p2y)) {
                            
                        lookaheadVector.x() = x2 + x;
                        lookaheadVector.y() = y2 + y;
                    }
                }
            }
        } 
        return lookaheadVector;
    }

private:

    millimeter_t m_lookAheadDistance;                 // the distance to look ahead
    millimeter_t m_wheelBase;                         // length between wheel bases
    std::vector<Vector2<millimeter_t>> m_wayPoints;   // list of waypoint coordinates

    // computes the turning angle using the lookahead point
    degree_t computeTurningAngle(const millimeter_t xrobot, const millimeter_t yrobot 
                                ,const millimeter_t xlookahead, const millimeter_t ylookahead
                                ,const radian_t heading) {
        using namespace units::math;
        // calculating bearing [robot-lookahead point]
        const auto dx = xlookahead-xrobot;
        const auto dy = ylookahead-yrobot;
        const degree_t bearing = atan2(dy,dx)-heading;
       
        // calculating turning angle
        const auto xlength = sin(bearing) * m_lookAheadDistance;

        // calculating arc to turn
        const auto kb = (2*xlength*m_wheelBase)/(pow<2>(m_lookAheadDistance));
        const degree_t turningAngle = atan(kb);
        return turningAngle;
    }

    // sign function
    template <typename T>static int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
}; // PurePursuitController
}  // Robot
}  // BoBRobotics




