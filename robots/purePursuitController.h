// purePursuitController.h

// BoB robotics includes
#include "common/main.h"
#include "robots/car_display.h"

// Third-party includes
#include "third_party/units.h"


// Standard C++ includes
#include <chrono>

#include <iostream>
#include <thread>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace BoBRobotics {
namespace Robots {
class PurePursuitController {

    using meter_t = units::length::meter_t;
    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using second_t = units::time::second_t;

public:

    PurePursuitController() {

    }

    PurePursuitController(units::length::millimeter_t lookahead) : lookAheadDistance(lookahead) {

    }

    void setWayPoints(std::vector<std::vector<millimeter_t>> wp) {
        wayPoints = wp;
    }


    void getTurningAngle(millimeter_t x, millimeter_t y) {
        getLookAheadPoint(x,y, lookAheadDistance);
    }


    std::vector<millimeter_t> getLookAheadPoint(millimeter_t x, millimeter_t y, millimeter_t r) {
        std::vector<millimeter_t> lookahead;

         if (wayPoints.size() > 1) {
            for (unsigned int i = 0; i < wayPoints.size()-1; i++) {
                // path points
                std::vector<millimeter_t> segmentStart = wayPoints.at(i);
                std::vector<millimeter_t> segmentEnd = wayPoints.at(i+1);

                std::vector<millimeter_t> p1,p2;
                auto p1x = segmentStart.at(0) - x;
                auto p1y = segmentStart.at(1) - y;
                auto p2x = segmentEnd.at(0) - x;
                auto p2y = segmentEnd.at(1) - y;

                // intersection point with circle
                auto dx = p2x - p1x;
                auto dy = p2y - p1y;
                auto d = units::math::sqrt(dx * dx + dy * dy);
                auto D = p1x * p2y - p2x * p1y;

                // if the discriminant is zero -> no intersection
                auto discriminant = r * r * d * d - D * D;
                if (discriminant.value() < 0) continue;

                // x components of the intersection point
                auto x1 = (D * dy + sgn(dy.value()) * dx * units::math::sqrt(discriminant)) / (d*d);
                auto x2 = (D * dy - sgn(dy.value()) * dx * units::math::sqrt(discriminant)) / (d*d);
                // y components of the intersection point
                auto y1 = (-D * dx + units::math::fabs(dy) * units::math::sqrt(discriminant)) / (d*d);
                auto y2 = (-D * dx - units::math::fabs(dy) * units::math::sqrt(discriminant)) / (d*d);

                // if there is 2 intersections possible with the circle, we select the second one, 
                // as that will be closer to the end point of the segment
                bool validIntersection1 = units::math::min(p1x, p2x) < x1 && x1 < units::math::max(p1x, p2x) ||
                                          units::math::min(p1y, p2y) < y1 && y1 < units::math::max(p1y,p2y);
                bool validIntersection2 = units::math::min(p1x, p2x) < x2 && x2 < units::math::max(p1x, p2x) ||
                                          units::math::min(p1y, p2y) < y2 && y2 < units::math::max(p1y,p2y);

                if (validIntersection1 || validIntersection2) lookahead.clear(); 
                if (validIntersection1) {
                    lookahead.push_back((x1 + x));
                    lookahead.push_back((y1 + y));
                }

                if (validIntersection2) {
                    if (lookahead.empty() || units::math::fabs(x1 - p2x) > units::math::fabs(x2 - p2x) ||
                        units::math::fabs(y1 - p2y) > units::math::fabs(y2 - p2y)) {
                            
                        lookahead.clear();
                        lookahead.push_back((x2 + x)); 
                        lookahead.push_back((y2 + y));
                    }
                }
            }
            return lookahead;

        }
    }

private:

    millimeter_t lookAheadDistance;
    std::vector<std::vector<millimeter_t>> wayPoints;


    



}; // PurePursuitController
}  // Robot
}  // BoBRobotics




