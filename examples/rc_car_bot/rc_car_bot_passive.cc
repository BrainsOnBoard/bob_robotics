// BoB robotics includes
#include "common/stopwatch.h"
#include "robots/rc_car_bot.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace units::literals;
using namespace std::literals;

int bobMain(int, char**)
{
    PassiveRCCarBot bot;

    while (true) {
        const auto move = bot.readRemoteControl();
        std::cout << move.first << ", " << move.second << "\n";

        std::this_thread::sleep_for(250ms);
    }

    return EXIT_SUCCESS;
}
