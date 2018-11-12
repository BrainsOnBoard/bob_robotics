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

auto now()
{
    return std::chrono::high_resolution_clock::now();
}

int main()
{
    Robots::Simulator simulator;
    simulator.setRobotSize(16.4_cm, 35_cm);

    auto lastTime = now();
    while (!simulator.didQuit()) {
        // Get time since last frame
        const auto currentTime = now();
        const auto dt = currentTime - lastTime;
        simulator.simulationStep(0_mps, 0_deg_per_s, dt);
        lastTime = currentTime;

        // Put a small delay in
        std::this_thread::sleep_for(10ms);
    }
}
