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
    double runTimeSecs{ 30 };
    CLI::App app{ "Record data with RC car robot" };
    auto *useSystemClock = app.add_flag("--use-system-clock",
                                        "Use the system clock to get current date rather than GPS");
    app.add_option("--runtime", runTimeSecs, "Time to run for");
    CLI11_PARSE(app, argc, argv);
    LOGI << "Running for " << runTimeSecs << "s\n";

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
    while (recorder.step(robot, imu) < second_t{ runTimeSecs }) {
        catcher.check();
    }

    return EXIT_SUCCESS;
}
