// BoB robotics includes
#include "common/main.h"
#include "robots/simulatedAckermanCar.h"
#include "robots/car_display.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>

#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;

int
bob_main(int, char **)
{
    //Robots::SimulatedTank<> robot(1.4_mps, 500_mm); // Tank agent
    Robots::SimulatedAckermanCar<> car(1.4_mps, 500_mm);
    Robots::CarDisplay display(10.2_m,160_mm);                     // For displaying the agent

    auto mmps = 1_mps;
    units::angle::degree_t deg = units::angle::degree_t(20);
    while(display.isOpen()) {

        
        car.move(mmps, deg);
        auto key = display.runGUI(car.getPose());
        
        
        if (key.first == SDLK_UP) {
            mmps = 1.4_mps;
        } 

        if (key.first == SDLK_DOWN) {
            mmps = 0_mps;
        }

        if (key.first == SDLK_LEFT) {
            deg = units::angle::degree_t(30);
        }

        else if (key.first == SDLK_RIGHT) {
            deg = units::angle::degree_t(-30);
        }

        else {
            deg = units::angle::degree_t(0);
        }
        
    }
    return EXIT_SUCCESS;
}
