// BoB robotics includes
#include "plog/Log.h"
#include "common/robo_claw.h"

#include <chrono>
#include <thread>

using namespace BoBRobotics;

int bobMain(int, char **)
{
    using namespace std::chrono_literals;

    // Create roboclaw interface
    RoboClaw roboClaw;

    std::cout << roboClaw.getVersion() << std::endl;

    roboClaw.setMotor1Speed(0.1f);
    for(int i = 0; i < 100; i++) {
        std::cout << roboClaw.getMotor1Speed() << "," << roboClaw.getMotor1Encoder() << std::endl;
        std::this_thread::sleep_for(50ms);
    }
    roboClaw.setMotor1Speed(0.0f);
    return 0;
}
