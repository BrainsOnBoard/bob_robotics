#include "robot_recorder.h"

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "robots/ackermann/rc_car_bot.h"

// Third-party includes
#include "third_party/CLI11.hpp"

using namespace BoBRobotics;
using namespace units::time;
using namespace units::literals;

int
bobMain(int argc, char *argv[])
{
    CLI::App app{ "Record data with RC car robot" };
    auto *useSystemClock = app.add_flag("--use-system-clock",
                    "Use the system clock to get current date rather than GPS");
    app.allow_extras();
    CLI11_PARSE(app, argc, argv);

    // Time to run data collection for
    millisecond_t runTime;
    switch (app.remaining_size()) {
    case 0:
        runTime = 30_s;
        break;
    case 1:
        runTime = second_t{ std::stod(app.remaining()[0]) };
        break;
    default:
        std::cout << app.help();
        return EXIT_FAILURE;
    }
    LOGD << "running for " << runTime;

    // setting up
    Robots::Ackermann::PassiveRCCarBot robot;
    GPS::GPSReader gps;
    BN055 imu;
    RobotRecorder recorder{ gps, useSystemClock->count() > 0 };

    // Catch Ctrl+C etc.
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals();

    // Keep recording readings from sensors until runTime has passed
    LOGI << "RECORDING STARTED";
    while (recorder.step(robot, imu) < runTime) {
        catcher.check();
    }

    return EXIT_SUCCESS;
}
