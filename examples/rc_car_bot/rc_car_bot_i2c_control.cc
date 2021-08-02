// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/stopwatch.h"
#include "robots/ackermann/rc_car_bot.h"

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
    Ackermann::RCCarBot bot;
    Stopwatch sw;

    std::cout << "Press Ctrl+C to quit\n";
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals();

    while (true) {
        sw.start();
        catcher.check();

        const auto move = bot.readRemoteControl();
        bot.move(move.first, move.second);
        std::cout << move.first << ", " << move.second << "\n";

        // Throttle motor commands otherwise the robot starts wigging out
        std::this_thread::sleep_for(30ms);
    }

    return EXIT_SUCCESS;
}
