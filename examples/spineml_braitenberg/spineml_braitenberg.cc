// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/simulated_tank.h"
#include "common/sfml_world.h"

// SpineML simulator includes
#include "simulator.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;

int bob_main(int, char **)
{
    Robots::SimulatedTank<> robot(0.3_mps, 104_mm); // Tank agent
    SFMLWorld<> display;                            // For displaying the agent
    auto car = display.createCarAgent();

    SpineMLSimulator::Simulator simulator("experiment1.xml", ".");
    while(true) {
        simulator.stepTime();

        // Refresh display
        car.setPose(robot.getPose());
        display.update(car);

    }

    return EXIT_SUCCESS;
}
