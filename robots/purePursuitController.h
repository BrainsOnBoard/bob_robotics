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
        getLookaheadPoint(x,y);
    }

private:

    millimeter_t lookAheadDistance;
    std::vector<std::vector<millimeter_t>> wayPoints;


    void getLookaheadPoint(millimeter_t x, millimeter_t y) {

        std::vector<millimeter_t> lookAheadPoint;
        millimeter_t lookaheadPointX, lookaheadPointY;
        
        if (wayPoints.size() > 1) {
            for (unsigned int i = 0; i < wayPoints.size()-1; i++) {

                // path points
                std::vector<millimeter_t> lineStartPoints = wayPoints.at(i);
                std::vector<millimeter_t> lineEndPoints = wayPoints.at(i+1);

                //translated coordinates 
                millimeter_t translatedCoordX = lineEndPoints.at(0) - lineStartPoints.at(0);
                millimeter_t translatedCoordY = lineEndPoints.at(1) - lineStartPoints.at(1);

                millimeter_t translatedRobotCoordX = x - lineStartPoints.at(0);
                millimeter_t translatedRobotCoordY = y - lineStartPoints.at(1);

                auto angle = units::math::atan2(translatedCoordY, translatedCoordX);
                auto turnedCoordX = translatedCoordX * units::math::cos(angle) - translatedCoordY * units::math::sin(angle);
                auto turnedCoordY = 0;

                auto turnedRobotCoordX = translatedRobotCoordX * units::math::cos(angle) - translatedRobotCoordY * units::math::sin(angle);
                auto turnedRobotCoordY = translatedRobotCoordY * units::math::cos(angle) + translatedRobotCoordX * units::math::sin(angle);


                auto intersectingPointX = turnedRobotCoordX + units::math::sqrt(lookAheadDistance * lookAheadDistance - turnedRobotCoordY * turnedRobotCoordY);

                // if the point is on the rotated lines 
                if ((intersectingPointX.value() > 0) && (intersectingPointX < turnedCoordX)) {
                    lookaheadPointX = intersectingPointX * units::math::cos(-angle) + lineStartPoints.at(0);
                    lookaheadPointY = intersectingPointX * units::math::sin(-angle) + lineStartPoints.at(1);

                    std::cout << " x " << lookaheadPointX << " y " << lookaheadPointY << std::endl;
                }

            }
        }
    }



}; // PurePursuitController
}  // Robot
}  // BoBRobotics