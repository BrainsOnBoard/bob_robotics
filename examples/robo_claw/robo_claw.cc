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
    std::cout << "Version:" << roboClaw.getVersion() << std::endl;
    std::cout << "Battery voltage:" << roboClaw.getBatteryVoltage() << "V" << std::endl;
    for(int i = 0; i < 100; i++) {
        if((i % 20) == 0) {
            roboClaw.setMotor1Speed(((i / 20) + 1) * 0.1f);
        }
        std::cout << "Speed:" << roboClaw.getMotor1Speed() << std::endl;
        std::this_thread::sleep_for(50ms);
    }
    roboClaw.setMotor1Speed(0.0f);
    return 0;
}
