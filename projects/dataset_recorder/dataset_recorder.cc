#include "robot_recorder.h"

// BoB robotics includes
#include "robots/ackermann/rc_car_bot.h"

using namespace BoBRobotics;
using namespace units::time;

int
bobMain(int argc, char *argv[])
{
    // setting up
    Robots::Ackermann::PassiveRCCarBot robot;
    RobotRecorder recorder;

    const millisecond_t runTime = second_t{ (argc > 1) ? std::stod(argv[1]) : 30.0 };
    LOGD << "running for " << runTime;

    // Keep recording readings from sensors until runTime has passed
    while (recorder.step(robot) < runTime)
        ;

    return EXIT_SUCCESS;
}
