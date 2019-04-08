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

// sign function
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace BoBRobotics {
namespace Robots {

using namespace units::math;
using namespace units::literals;

class PurePursuitController {

    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using radian_t = units::angle::radian_t;
   
public:

    PurePursuitController() {

    }

    PurePursuitController(millimeter_t lookahead, millimeter_t wheelBaseLength) : 
                          lookAheadDistance(lookahead), wheelBase(wheelBaseLength) {

    }

    //! set waypoints which forms a path to be followed
    void setWayPoints(std::vector<Vector2<millimeter_t>> wp) {
        wayPoints = wp;
    }

    //! adds to the list of waypoints
    void addToWayPoint(Vector2<millimeter_t> wayPoint) {
        wayPoints.push_back(wayPoint);
    }

    //! sets the lookahead distance. Large values causes the car to cut corners,
    //! where small values makes the car follow the path more closely
    void setLookAheadDistance(millimeter_t distance) {
        lookAheadDistance = distance;
    }

    //! sets the distance between the car's wheel bases
    void setWheelBaseLength(millimeter_t length) {
        wheelBase = length;
    }
  
    //! calculates the turning angle needed to follow the path 
    degree_t getTurningAngle(millimeter_t x, millimeter_t y, radian_t heading) {
        std::vector<millimeter_t> lookPoint = getLookAheadPoint(x,y, lookAheadDistance);
        if (!lookPoint.empty()) return computeTurningAngle(x,y, lookPoint.at(0), lookPoint.at(1), heading);
        return 0_deg; 
    }

    //! calculates the look-ahead point the robot follows
    std::vector<millimeter_t> getLookAheadPoint(millimeter_t x, millimeter_t y, millimeter_t r) {
        std::vector<millimeter_t> lookahead;

        if (wayPoints.size() > 1) {
            for (unsigned int i = 0; i < wayPoints.size()-1; i++) {

                // path points
                Vector2<millimeter_t> segmentStart = wayPoints.at(i);
                Vector2<millimeter_t> segmentEnd = wayPoints.at(i+1);

                std::vector<millimeter_t> p1,p2;
                auto p1x = segmentStart.x() - x;
                auto p1y = segmentStart.y() - y;
                auto p2x = segmentEnd.x() - x;
                auto p2y = segmentEnd.y() - y;

                // intersection point with circle
                auto dx = p2x - p1x;
                auto dy = p2y - p1y;
                auto d = sqrt(dx * dx + dy * dy);
                auto D = p1x * p2y - p2x * p1y;

                // if the discriminant is zero -> no intersection
                auto discriminant = r * r * d * d - D * D;
                if (discriminant.value() < 0) continue;

                // x components of the intersection point
                auto x1 = (D * dy + sgn(dy.value()) * dx * sqrt(discriminant)) / (d*d);
                auto x2 = (D * dy - sgn(dy.value()) * dx * sqrt(discriminant)) / (d*d);
                // y components of the intersection point
                auto y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / (d*d);
                auto y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / (d*d);

                // if there is 2 intersections possible with the circle, we select the second one, 
                // as that will be closer to the end point of the segment
                bool validIntersection1 = (min(p1x, p2x) < x1 && x1 < max(p1x, p2x)) ||
                                          (min(p1y, p2y) < y1 && y1 < max(p1y,p2y));
                bool validIntersection2 = (min(p1x, p2x) < x2 && x2 < max(p1x, p2x)) ||
                                          (min(p1y, p2y) < y2 && y2 < max(p1y,p2y));

                // we always want the latest path segment point so if we have a 
                // valid point, we delete the previous point
                if (validIntersection1 || validIntersection2) lookahead.clear(); 
                if (validIntersection1) {
                    lookahead.push_back((x1 + x));
                    lookahead.push_back((y1 + y));
                }

                // if there is a valid 2. intersection point, we keep that and remove
                // the first one
                if (validIntersection2) {
                    if (lookahead.empty() || fabs(x1 - p2x) > fabs(x2 - p2x) ||
                        fabs(y1 - p2y) > fabs(y2 - p2y)) {
                            
                        lookahead.clear();
                        lookahead.push_back((x2 + x)); 
                        lookahead.push_back((y2 + y));
                    }
                }
            }
            return lookahead;
        }
        return lookahead;
    }

private:

    millimeter_t lookAheadDistance;                 // the distance to look ahead
    millimeter_t wheelBase;                         // length between wheel bases
    std::vector<Vector2<millimeter_t>> wayPoints;   // list of waypoint coordinates

    // computes the turning angle using the lookahead point
    degree_t computeTurningAngle(millimeter_t xrobot, millimeter_t yrobot 
                            ,millimeter_t xlookahead, millimeter_t ylookahead, radian_t heading) {

        // calculating bearing [robot-lookahead point]
        auto dx = xlookahead-xrobot;
        auto dy = ylookahead-yrobot;
        degree_t bearing = atan2(dy,dx)-heading;
       
        // calculating turning angle
        auto xlength = sin(bearing) * lookAheadDistance;

        // calculating arc to turn
        auto kb = (2*xlength*wheelBase)/(pow<2>(lookAheadDistance));
        degree_t turningAngle = atan(kb);
        return turningAngle;
    }
}; // PurePursuitController
}  // Robot
}  // BoBRobotics




