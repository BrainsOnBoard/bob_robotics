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
    std::cout << roboClaw.getBatteryVoltage() << "V" << std::endl;
    roboClaw.setMotor1Speed(-0.1f);
    for(int i = 0; i < 100; i++) {

        //std::cout << "Query:";
        uint32_t speed = roboClaw.getMotor1Encoder();

        std::cout << std::endl << "Speed:" << speed << std::endl;
        //uint32_t encoder = roboClaw.getMotor1Encoder() << std::endl;
        std::this_thread::sleep_for(50ms);
    }
    roboClaw.setMotor1Speed(0.0f);
    return 0;
}
