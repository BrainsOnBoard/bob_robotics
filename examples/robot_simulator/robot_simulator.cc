// BoB robotics includes
#include "robots/simulator.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;

int main()
{
    Robots::Simulator simulator;
    simulator.setRobotSize(16.4_cm, 35_cm);
    while (!simulator.didQuit()) {
        simulator.simulationStep(1_mps, 100_deg_per_s, 10_ms);
        std::this_thread::sleep_for(10ms);
    }
}
