/*
    Small example program to demonstrate the Pure Pursuit Controller. The controller
    takes a list of coordinates as waypoint and calculates a desired turning angle
    to steer the car for a smooth path tracking
*/

// BoB robotics includes
#include "common/main.h"
#include "robots/simulatedAckermanCar.h"
#include "robots/car_display.h"
#include "robots/purePursuitController.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::length;
using namespace units::angle;

int bob_main(int, char **)
{

    Robots::SimulatedAckermanCar<> car(1.4_mps, 500_mm);   // simulated car
    Robots::CarDisplay display(10.2_m,160_mm);             // For displaying the agent
    std::vector<SDL_Rect> rekt_list;                       // list of waypoints

    float currentX = 0, currentY = 0;                      // current mouse coordinate click

    // adding the first coordinate
    //-------------------------------------------------------------------
    millimeter_t xMM, yMM;
    display.pixelToMM(currentX, currentY,  xMM,  yMM);
    std::vector<Vector2<millimeter_t>> wpCoordinates; 
    Vector2<millimeter_t> firstCoord(xMM, yMM);
    wpCoordinates.push_back(firstCoord);
    //-------------------------------------------------------------------

    auto mmps = 0_mps;    // speed of robot car
    degree_t deg = 0_deg; // angle of steering of robot car
    millimeter_t lookaheadDistance = 1000_mm; // lookahead distance

    auto wheelBase = car.getDistanceBetweenAxis(); // distance between wheel bases
    BoBRobotics::Robots::PurePursuitController controller(lookaheadDistance, wheelBase);

    while(display.isOpen()) {
        
        // each click will spawn a way point which is connected together 
        std::vector<float> mousePos = display.getMouseClickPixelPosition();
        float xp = mousePos[0];
        float yp = mousePos[1];
        
        // click to the screen to store waypoint in list
        if (xp != currentX && yp != currentY) {
            // draw a rectangle at the goal position
            SDL_Rect rekt;
            rekt.x = xp;
            rekt.y = yp;
            rekt.w = 5;
            rekt.h = 5;
            currentX = xp;
            currentY = yp;
            rekt_list.push_back(rekt); // save rectangle
            
            // we store the physical unit coordinate in the waypoint list
            display.pixelToMM(xp, yp,  xMM,  yMM);
            Vector2<millimeter_t> coords(xMM,yMM);
            wpCoordinates.push_back(coords);  
        }
        
        // set waypoints in controller
        controller.setWayPoints(wpCoordinates);

        // run a GUI step    
        auto key = display.runGUI(car.getPose());
        display.clearScreen();

        // draw all the waypoints on the screen
        for (unsigned int i =0; i < rekt_list.size(); i++) {
            display.drawRectangleAtCoordinates(rekt_list.at(i));
        }

        // draw lines between waypoints to form a path
        display.drawLinesBetweenRects(rekt_list, display.getRenderer());

        // draw lookahead point and a line to it from the robot
        std::vector<millimeter_t> lookPoint =  controller.getLookAheadPoint(car.getPose().x(), car.getPose().y(),lookaheadDistance);
        int pxx, pxy;
        auto robx = car.getPose().x();
        auto roby = car.getPose().y();
        auto heading = car.getPose().yaw();

        if (lookPoint.size() > 1) {
            display.mmToPixel(lookPoint.at(0),lookPoint.at(1),pxx,pxy);
            std::vector<SDL_Rect> rektVec;
            SDL_Rect rkt, rkt_rob;
            rkt.x = pxx;
            rkt.y = pxy;
            
            int rx,ry;
            display.mmToPixel(robx, roby, rx,ry);
            rkt_rob.x = rx;
            rkt_rob.y = ry;
            display.drawRectangleAtCoordinates(rkt);
            rektVec.push_back(rkt);
            rektVec.push_back(rkt_rob);
            // draw line between robot and lookahead point
            display.drawLinesBetweenRects(rektVec, display.getRenderer());
        }

        // calculate turning angle with controller
        auto turningAngle = controller.getTurningAngle(robx, roby, heading);
        degree_t turningAngleDeg = turningAngle;
    
        // if there is a key command, move car with keys, othwerwise listen to 
        //the controller command to turn the car so it follows the path
        if (key.first == SDLK_UP) {
            mmps = 1.4_mps; // go max speed
        } 
        else if (key.first == SDLK_DOWN) {
            mmps = 0_mps; // stop
        }

        // if left or right button pressed ->steering 30 degrees left and right, 
        // if no turn command pressed, turning is performed by the controller
        if (key.first == SDLK_LEFT) {
            deg = 30_deg;
            car.move(mmps, deg);
        }
        else if (key.first == SDLK_RIGHT) {
            deg = -30_deg;
            car.move(mmps, deg);
        }
        else {
            car.move(mmps, turningAngleDeg);
        }

    }
    return EXIT_SUCCESS;
}
