// BoB robotics includes
#include "robots/ev3/mindstorms_imu.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace units::angle;
using namespace units::angular_velocity;

int main()
{
    BoBRobotics::MindstormsIMU imu;
    degree_t angle;
    degrees_per_second_t velocity;
    while (true) {
        std::tie(angle, velocity) = imu.getYawAndVelocity();
        std::cout << "Angle: " << angle << "; velocity: " << velocity << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
