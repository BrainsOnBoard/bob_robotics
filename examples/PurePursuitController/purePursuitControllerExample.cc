// BoB robotics includes
#include "common/main.h"
#include "robots/simulatedAckermanCar.h"
#include "robots/car_display.h"

// Third-party includes
#include "third_party/units.h"

#include "robots/purePursuitController.h"

// Standard C++ includes
#include <chrono>

#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;

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



int
bob_main(int, char **)
{

    Robots::SimulatedAckermanCar<> car(1.4_mps, 500_mm);
    Robots::CarDisplay display(10.2_m,160_mm);                     // For displaying the agent
    std::vector<SDL_Rect> rekt_list;

    float currentX = 0, currentY = 0;


    //-------------------------------------------------------------------
    units::length::millimeter_t xMM, yMM;
    display.pixelToMM(currentX, currentY,  xMM,  yMM);
    std::vector<std::vector<units::length::millimeter_t>> wpCoordinates; 
    std::vector<units::length::millimeter_t> firstCoord;
    firstCoord.push_back(xMM);
    firstCoord.push_back(yMM);
    wpCoordinates.push_back(firstCoord);
    //-------------------------------------------------------------------

    auto mmps = 0_mps;
    units::angle::degree_t deg = units::angle::degree_t(0);

    auto wheelBase = car.getDistanceBetweenAxis();
    BoBRobotics::Robots::PurePursuitController controller(units::length::millimeter_t(2000), wheelBase);

    while(display.isOpen()) {
        
        // each click will spawn a way point which is connected together /////////
        std::vector<float> mousePos = display.getMouseClickPixelPosition();
        float xp = mousePos[0];
        float yp = mousePos[1];
        
        if (xp != currentX && yp != currentY) {
            // draw a rectangle at the goal position
            SDL_Rect rekt;
            rekt.x = xp;
            rekt.y = yp;
            rekt.w = 5;
            rekt.h = 5;
            currentX = xp;
            currentY = yp;

            rekt_list.push_back(rekt);
            
            
            display.pixelToMM(xp, yp,  xMM,  yMM);
            std::vector<units::length::millimeter_t> coords;
            coords.push_back(xMM);
            coords.push_back(yMM);
            wpCoordinates.push_back(coords);

            std::cout << " x " << xMM.value() << " y " << yMM.value() << std::endl;
        }
        //------------------------------------------------------------------------


        controller.setWayPoints(wpCoordinates);
        //controller.getTurningAngle(car.getPose().x(), car.getPose().y());


        
        auto key = display.runGUI(car.getPose());
        display.clearScreen();

        for (unsigned int i =0; i < rekt_list.size(); i++) {
            display.drawRectangleAtCoordinates(rekt_list.at(i));
        }


        // draw lines between waypoints
        drawLinesBetweenRects(rekt_list, display.getRenderer());


        // draw lookahead point
        std::vector<units::length::millimeter_t> lookPoint =  controller.getLookAheadPoint(car.getPose().x(), car.getPose().y(),units::length::millimeter_t(1000));
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
            drawLinesBetweenRects(rektVec, display.getRenderer());
        }
        auto turningAngle = controller.getTurningAngle(robx, roby, heading);
        units::angle::degree_t turningAngleDeg = turningAngle;
        

    

        if (key.first == SDLK_UP) {
            mmps = 1.4_mps;
        } 

        if (key.first == SDLK_DOWN) {
            mmps = 0_mps;
        }

        if (key.first == SDLK_LEFT) {
            deg = units::angle::degree_t(30);
            car.move(mmps, deg);
        }

        else if (key.first == SDLK_RIGHT) {
            deg = units::angle::degree_t(-30);
            car.move(mmps, deg);
        }

        else {
            deg = units::angle::degree_t(0);
            //auto heading = car.getPose().yaw();
            car.move(mmps, turningAngleDeg);
            std::cout << turningAngleDeg.value() << std::endl;
        }
        
    }
    return EXIT_SUCCESS;
}
