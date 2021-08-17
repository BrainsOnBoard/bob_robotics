#include "robot_recorder.h"

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "robots/ackermann/rc_car_bot.h"

using namespace BoBRobotics;
using namespace units::time;

int
bobMain(int argc, char *argv[])
{
    // setting up
    Robots::Ackermann::PassiveRCCarBot robot;
    GPS::GPSReader gps;
    BN055 imu;
    RobotRecorder recorder{ gps };

    const millisecond_t runTime = second_t{ (argc > 1) ? std::stod(argv[1]) : 30.0 };
    LOGD << "running for " << runTime;

    // Catch Ctrl+C etc.
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals();

    // Keep recording readings from sensors until runTime has passed
    while (recorder.step(robot, imu) < runTime) {
        catcher.check();
    }

    return EXIT_SUCCESS;
}
