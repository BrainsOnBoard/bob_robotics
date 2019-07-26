/*
    Small example program to demonstrate the Pure Pursuit Controller. The controller
    takes a list of coordinates as waypoint and calculates a desired turning angle
    to steer the car for a smooth path tracking.
    To create a path, press the left mouse button to place a waypoint to the screen
    making a path. Then, pressing the space button will start the controller algorithm
    to make the robot follow the path

    key commands:
        [    ARROW UP     ] = apply max speed to car
        [   ARROW DOWN    ] = stop car
        [   ARROW LEFT    ] = turn steering wheel by 30 degrees left
        [   ARROW RIGHT   ] = turn steering wheel by 30 degrees right
        [      SPACE      ] = toggle controller algorithm on/off
        [LEFT MOUSE BUTTON] = places a waypoint on the screen adding it to the path
*/

// BoB robotics includes
#include "common/main.h"
#include "robots/simulated_ackermann.h"
#include "robots/control/pure_pursuit_controller.h"
#include "viz/car_display/car_display.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::length;
using namespace units::angle;
using namespace units::velocity;

//! draw lines between a list of points
void drawLinesBetweenRects(std::vector<SDL_Rect> listRects, SDL_Renderer *renderer) {
    if (!listRects.empty()) {
        for (unsigned int i = 0; i < listRects.size()-1; i++) {
            SDL_Rect current_rectangle = listRects.at(i);
            SDL_Rect next_rectangle = listRects.at(i+1);

            float lineStartX = current_rectangle.x;
            float lineStartY = current_rectangle.y;
            float lineEndX = next_rectangle.x;
            float lineEndY = next_rectangle.y;

            SDL_RenderDrawLine(renderer, lineStartX, lineStartY, lineEndX, lineEndY);
        }
    }
}

int bob_main(int, char **)
{

    Robots::SimulatedAckermann car(1.4_mps, 500_mm); // simulated car
    Viz::CarDisplay display(10.2_m, 160_mm);         // For displaying the agent
    std::vector<SDL_Rect> rekt_list;                 // list of waypoints
    float currentX = 0, currentY = 0;                // current mouse coordinate click
    bool isControllerOn = true;

    // adding the first coordinate
    //-------------------------------------------------------------------
    millimeter_t xMM, yMM;
    display.pixelToMM(currentX, currentY,  xMM,  yMM);
    std::vector<Vector2<millimeter_t>> wpCoordinates;
    wpCoordinates.emplace_back(xMM, yMM);
    //-------------------------------------------------------------------

    auto mmps = 0_mps;    // speed of robot car
    degree_t deg = 0_deg; // angle of steering of robot car
    constexpr millimeter_t lookaheadDistance = 1000_mm; // lookahead distance
    constexpr meters_per_second_t max_speed = 1.4_mps;  // car's max speed
    constexpr millimeter_t stopping_dist = 1_cm;        // car's stopping distance

    const auto wheelBase = car.getDistanceBetweenAxis(); // distance between wheel bases
    Robots::PurePursuitController controller(lookaheadDistance, wheelBase, stopping_dist);

    while(display.isOpen()) {

        // each click will spawn a way point which is connected together
        std::vector<float> mousePos = display.getMouseClickPixelPosition();
        const float xp = mousePos[0];
        const float yp = mousePos[1];

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
            wpCoordinates.emplace_back(xMM,yMM);
        }

        // set waypoints in controller
        controller.setWayPoints(wpCoordinates);

        // run a GUI step
        const auto key = display.runGUI(car.getPose());

        // clear screen before drawing other elements
        display.clearScreen();

        // draw all the waypoints on the screen
        for (auto &r : rekt_list) {
            display.drawRectangleAtCoordinates(r);
        }

        // draw lines between waypoints to form a path
        if (rekt_list.size() > 1) {
            drawLinesBetweenRects(rekt_list, display.getRenderer());
        }

        // draw lookahead point and a line to it from the robot
        Vector2<millimeter_t> lookPoint;
        const bool didGetPoint = controller.getLookAheadPoint(car.getPose().x(), car.getPose().y(),lookaheadDistance,lookPoint);
        int pxx, pxy;
        const auto robx = car.getPose().x();
        const auto roby = car.getPose().y();
        const auto heading = car.getPose().yaw();

        // draw the line from robot to lookahead point if there is one
        if (lookPoint.size() > 1 && didGetPoint) {
            display.mmToPixel(lookPoint.x(),lookPoint.y(),pxx,pxy);
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
            rektVec.push_back(rkt_rob); // rect at robot
            // draw line between robot and lookahead point
            drawLinesBetweenRects(rektVec, display.getRenderer());
        }

        // calculate turning angle with controller
        degree_t turningAngle;
        const bool didGetAngle = controller.getTurningAngle(robx, roby, heading, turningAngle);


        // if there is a key command, move car with keys, othwerwise listen to
        // the controller command to turn the car so it follows the path
        if (key.second) {
            switch (key.first)
            {
                case SDLK_UP:
                    mmps = max_speed; // go max speed
                    deg = 0_deg;
                    break;
                case SDLK_DOWN:
                    mmps = 0_mps;     // stop
                    deg = 0_deg;
                    break;
                case SDLK_LEFT:
                    deg = 30_deg;
                    break;
                case SDLK_RIGHT:
                    deg = -30_deg;
                    break;
                // if user presses space, it toggles the controller on/off
                case SDLK_SPACE:
                    if (isControllerOn) {
                        mmps = 0_mps;
                        deg = 0_deg;
                    } else {
                        mmps = max_speed;
                        deg = 0_deg;
                        controller.setlookAheadDistance(lookaheadDistance); // reset lookahead distance
                    }
                    isControllerOn = !isControllerOn;
                    break;
                default:
                    break;
            }
        }
        // if controller is on -> car moves with controller's commands
        if (isControllerOn) {
            if (didGetAngle) {
                car.move(mmps, turningAngle);
            } else {
                car.move(0_mps, 0_deg); // stop car
            }
        } else {
        // if controller is off -> car moves with user's commands
            car.move(mmps, deg);
        }
    }
    return EXIT_SUCCESS;
}
