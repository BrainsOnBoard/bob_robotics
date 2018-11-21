// BoB robotics includes
#include "viz/agent_renderer.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

int
main()
{
    Viz::AgentRenderer renderer;

    // run the program as long as the window is open
    while (renderer.isOpen()) {
        renderer.update();
        std::this_thread::sleep_for(5ms);
    }
}