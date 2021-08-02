// BoB robotics includes
#include "robots/ackermann/rc_car_bot.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::literals;
using namespace std::literals;

int bobMain(int, char **)
{
    Robots::RCCarBot bot;

    // Drive in a circle for 10s
    bot.move(0.5f, -35_deg);
    std::this_thread::sleep_for(10s);

    return EXIT_SUCCESS;
}
