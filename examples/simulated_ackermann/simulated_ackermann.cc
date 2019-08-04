// BoB robotics includes
#include "common/main.h"
#include "robots/simulated_ackermann.h"
#include "viz/car_display/car_display.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::angle;
using namespace units::length;

int bob_main(int, char **)
{
    Robots::SimulatedAckermann car(1.4_mps, 500_mm); // simulated ackermann car
    Viz::CarDisplay display(10.2_m, 160_mm);         // For displaying the agent

    auto mmps = 0_mps;
    degree_t deg = 0_deg;
    while(display.isOpen()) {
        // clear screen
        display.clearScreen();

        //move car
        car.move(mmps, deg);

        // run GUI 1 step and get user key command
        const auto key = display.runGUI(car.getPose());

        // if key up -> add speed
        if (key.first == SDLK_UP) {
            mmps = 1.4_mps;
        }

        // if key down -> stop
        if (key.first == SDLK_DOWN) {
            mmps = 0_mps;
        }

        // if left or right key -> turn steering wheel
        if (key.first == SDLK_LEFT) {
            deg = 30_deg;
        }

        else if (key.first == SDLK_RIGHT) {
            deg = -30_deg;
        }

        else {
            deg = 0_deg;
        }

    }
    return EXIT_SUCCESS;
}